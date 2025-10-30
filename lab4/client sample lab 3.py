#python 3 code
import socket
from time import *
from pynput import keyboard
"""pynput: On Mac OSX, one of the following must be true:
* The process must run as root. OR
* Your application must be white listed under Enable access for assistive devices. Note that this might require that you package your application, since otherwise the entire Python installation must be white listed."""
import sys
import threading
import enum
import urllib.request
import cv2
import numpy
import copy

socketLock = threading.Lock()
imageLock = threading.Lock()

IP_ADDRESS = "192.168.1.102" 	# SET THIS TO THE RASPBERRY PI's IP ADDRESS
RESIZE_SCALE = 2 # try a larger value if your computer is running slow.
ENABLE_ROBOT_CONNECTION = True

# You should fill this in with your states
class States(enum.Enum):
    SEARCH = enum.auto()
    VISIBLE = enum.auto()
    TURN_L = enum.auto()
    TURN_R = enum.auto()
    FORWARD = enum.auto()
    FAR = enum.auto()
    MEDIUM = enum.auto()
    CLOSE = enum.auto()

    FIND = enum.auto()
    CHASE = enum.auto()

class StateMachine(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)   # MUST call this to make sure we setup the thread correctly
        # CONFIGURATION PARAMETERS
        global IP_ADDRESS
        self.IP_ADDRESS = IP_ADDRESS
        self.CONTROLLER_PORT = 5001
        self.TIMEOUT = 10					# If its unable to connect after 10 seconds, give up.  Want this to be a while so robot can init.
        self.STATE = States.FIND
        self.RUNNING = True
        self.DIST = False
        self.video = ImageProc()
        self.leftScreen = 320 / 3
        self.rightScreen = self.leftScreen * 2
        self.topScreen = 2 * 240 / 5
        self.bottomScreen = 3 * 240 / 5

        self.screenW = 320
        self.screenH = 240

        # Start video
        self.video.start()
        
        # connect to the motorcontroller
        try:
            with socketLock:
                self.sock = socket.create_connection( (self.IP_ADDRESS, self.CONTROLLER_PORT), self.TIMEOUT)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("Connected to RP")
        except Exception as e:
            print("ERROR with socket connection", e)
            sys.exit(0)
    
        # connect to the robot
        """ The i command will initialize the robot.  It enters the create into FULL mode which means it can drive off tables and over steps: be careful!"""
        if ENABLE_ROBOT_CONNECTION:
            with socketLock:
                self.sock.sendall("i /dev/ttyUSB0".encode())
                print("Sent command")
                result = self.sock.recv(128)
                print(result)
                if result.decode() != "i /dev/ttyUSB0":
                    self.RUNNING = False
        
        self.sensors = Sensing(self.sock)
        # Start getting data
        if ENABLE_ROBOT_CONNECTION:
            self.sensors.start()
        
        # Collect events until released
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
            
            
    def run(self):

        # BEGINNING OF THE CONTROL LOOP
        while self.RUNNING:
            sleep(0.1)

            if self.STATE == States.FIND:
                print("Ferb is going to find you...")
                
                if self.video.visible:              # found ball
                    self.STATE = States.VISIBLE

                else:                               # ball is not seen
                    with socketLock:
                        self.sock.sendall("a spin_left(50)".encode())
                        self.sock.recv(128)
            # elif self.STATE == States.CHASE:
            #     print("Ferb has found you, run.")
                
            #     if self.video.visible == False:
            #         self.STATE = States.VISIBLE

            #     speed = 0

            #     speed = None # put equation here that weighs objects in vision!!!
            
            #Start
            elif self.STATE == States.CHASE:
                print("Ferb: Chasing the beach ball while navigating between cones!")

                # --- Check visibility ---
                if not self.video.visible:
                    print("Ferb lost the ball! Searching again...")
                    self.STATE = States.SEARCH
                    continue

                # --- Extract object positions ---
                ball_x, ball_y = self.video.objCentroid if self.video.objCentroid else (0, 0)
                green_x, _ = self.video.greenCenter if self.video.greenCenter else (0, 0)
                red_x, _ = self.video.redCenter if self.video.redCenter else (0, 0)
                screen_center = self.screenW / 2

                # --- Determine lane center (from cones) ---
                if green_x > 0 and red_x > 0:
                    lane_center = (green_x + red_x) / 2
                else:
                    lane_center = screen_center  # if one cone missing, drive straight

                # --- Combine both influences (ball + lane) ---
                # Give more weight to the ball, but still align with cones
                if ball_x > 0:
                    target_x = (0.7 * ball_x) + (0.3 * lane_center)
                else:
                    target_x = lane_center

                # --- Compute steering adjustment ---
                error = target_x - screen_center   # positive = ball is to the right
                steering = int(error * 0.5)        # proportional control (gain = 0.5)
                steering = max(min(steering, 100), -100)  # clamp to safe range

                # --- Forward speed adjustment based on distance ---
                if ball_y < self.topScreen:
                    forward_speed = 80   # close → slow down
                elif ball_y > self.bottomScreen:
                    forward_speed = 180  # far → speed up
                else:
                    forward_speed = 130  # medium distance

                # --- Cone avoidance boost ---
                # If too close to a cone, bias steering away
                cone_push = 40
                if green_x > 0 and ball_x < green_x + 60:
                    print("Ferb: Too close to left (green) cone → steering right!")
                    steering += cone_push
                if red_x > 0 and ball_x > red_x - 60:
                    print("Ferb: Too close to right (red) cone → steering left!")
                    steering -= cone_push

                # Clamp again after adjustments
                steering = max(min(steering, 100), -100)

                # --- Send drive command ---
                command = f"a drive({forward_speed},{-steering})"
                with socketLock:
                    self.sock.sendall(command.encode())
                    self.sock.recv(128)

                # Debug info
                print(f"Ball at x={ball_x}, Green={green_x}, Red={red_x}, "
                    f"Steering={steering}, Speed={forward_speed}")

            #finish



            # starting state to look for yellow beach ball
            if self.STATE == States.SEARCH:
                print("Ferb: SEARCHING")
                
                if self.video.visible:              # found ball
                    self.STATE = States.VISIBLE

                else:                               # ball is not seen
                    with socketLock:
                        self.sock.sendall("a spin_left(50)".encode())
                        self.sock.recv(128)

            # state to decide next state
            elif self.STATE == States.VISIBLE:
                with socketLock:                                    # stop robot
                    self.sock.sendall("a drive_straight(0)".encode())
                    self.sock.recv(128)

                if self.video.objCentroid[0] < self.leftScreen:           # left sixth of screen
                    self.STATE = States.TURN_L
                    print("Ferb: LEFT!!!")

                elif self.video.objCentroid[0] > self.rightScreen:     # right sixth of screen
                    self.STATE = States.TURN_R
                    print("Ferb: RIGHT!!!")

                elif self.video.visible:                            # drive forward
                    self.STATE = States.FORWARD
                    print("Ferb: FORWARD!!!")

                else:                                               # no ball; search
                    self.STATE = States.SEARCH

            
            elif self.STATE == States.TURN_L:
                
                with socketLock:
                    self.sock.sendall("a spin_left(50)".encode())
                    self.sock.recv(128)

                # check if state should be changed
                if self.video.visible != True:
                    self.STATE = States.SEARCH

                elif self.video.objCentroid[0] > self.leftScreen:
                    self.STATE = States.VISIBLE

            elif self.STATE == States.TURN_R:

                with socketLock:
                    self.sock.sendall("a spin_right(50)".encode())
                    self.sock.recv(128)

                # check if state should be changed
                if self.video.visible != True:
                    self.STATE = States.SEARCH

                elif self.video.objCentroid[0] < self.rightScreen:
                    self.STATE = States.VISIBLE
            
            elif self.STATE == States.FORWARD:
                
                # check which forward state
                if self.video.visible != True:
                    self.STATE = States.SEARCH

                elif self.video.objCentroid[0] < self.leftScreen or self.video.objCentroid[0] > self.rightScreen:
                    self.STATE = States.VISIBLE

                elif self.video.objCentroid[1] < self.topScreen:
                    self.STATE = States.CLOSE
                    print("Ferb: CLOSE!!!")

                elif self.video.objCentroid[1] > self.bottomScreen:
                    self.STATE = States.FAR
                    print("Ferb: FAR!!!")

                else:
                    self.STATE = States.MEDIUM
                    print("Ferb: MEDIUM!!!")

                
            # forward states
            if self.STATE == States.MEDIUM:

                with socketLock:
                    self.sock.sendall("a drive_straight(100)".encode())
                    self.sock.recv(128)

                self.STATE = States.FORWARD

            elif self.STATE == States.FAR:

                with socketLock:
                    self.sock.sendall("a drive_straight(150)".encode())
                    self.sock.recv(128)

                self.STATE = States.FORWARD

            elif self.STATE == States.CLOSE:
                
                with socketLock:
                    self.sock.sendall("a drive_straight(50)".encode())
                    self.sock.recv(128)

                self.STATE = States.FORWARD

            # TODO: Work here


        # END OF CONTROL LOOP
        
        # First stop any other threads talking to the robot
        self.sensors.RUNNING = False
        self.video.RUNNING = False
        
        sleep(1)    # Wait for threads to wrap up
        
        # Need to disconnect
        """ The c command stops the robot and disconnects.  The stop command will also reset the Create's mode to a battery safe PASSIVE.  It is very important to use this command!"""
        with socketLock:
            self.sock.sendall("c".encode())
            print(self.sock.recv(128))
            self.sock.close()

        # If the user didn't request to halt, we should stop listening anyways
        self.listener.stop()

        #self.sensors.join()
        #self.video.join()

    def on_press(self, key):
        # WARNING: DO NOT attempt to use the socket directly from here
        try:
            print('alphanumeric key {0} pressed'.format(key.char))
            if key.char == 'q':
                # Stop listener
                self.RUNNING = False
                self.sensors.RUNNING = False
                self.video.RUNNING = False
        except AttributeError:
            print('special key {0} pressed'.format(key))

    def on_release(self, key):
        # WARNING: DO NOT attempt to use the socket directly from here
        print('{0} released'.format(key))
        if key == keyboard.Key.esc or key == keyboard.Key.ctrl:
            # Stop listener
            self.RUNNING = False
            self.sensors.RUNNING = False
            self.video.RUNNING = False
            return False

# END OF STATEMACHINE


class Sensing(threading.Thread):
    def __init__(self, socket):
        threading.Thread.__init__(self)   # MUST call this to make sure we setup the thread correctly
        self.RUNNING = True
        self.sock = socket
    
    def run(self):
        while self.RUNNING:
            sleep(1)
            # This is where I would get a sensor update
            # Store it in this class
            # You can change the polling frequency to optimize performance, don't forget to use socketLock
            with socketLock:
                # self.sock.sendall("a distance".encode())
                # print(self.sock.recv(128))
                pass


# END OF SENSING

class ImageProc(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)   # MUST call this to make sure we setup the thread correctly
        global IP_ADDRESS
        self.IP_ADDRESS = IP_ADDRESS
        self.PORT = 8081
        self.RUNNING = True
        self.latestImg = []
        self.feedback = []
        self.thresholds = {'lo_hue':0,'lo_saturation':0,'lo_value':0,'hi_hue':0,'hi_saturation':0,'hi_value':0}
        self.objCentroid = (0,0)
        self.visible = False

        self.greenCenter = (0,0)
        self.redCenter = (0,0)

        # hard code for yellow beach ball
        # this masks out a yellow beachball
        self.yellowBeachball = {'lo_hue':0,'lo_saturation':115,'lo_value':130,'hi_hue':90,'hi_saturation':185,'hi_value':230}

        # this masks out a green cone
        self.greenCone = {'lo_hue':38,'lo_saturation':127,'lo_value':64,'hi_hue':124,'hi_saturation':208,'hi_value':111}

        # this mask out a red cone
        self.redCone = {'lo_hue':0,'lo_saturation':56,'lo_value':141,'hi_hue':88,'hi_saturation':116,'hi_value':227}

        """
        self.thresholds = {'low_red':0,'high_red':0,'low_green':0,'high_green':0,'low_blue':0,'high_blue':0}
        """

    def run(self):
        url = "http://"+self.IP_ADDRESS+":"+str(self.PORT)
        stream = urllib.request.urlopen(url)
        while(self.RUNNING):
            sleep(0.1)
            bytes = b''
            while self.RUNNING:
                bytes += stream.read(8192)  #image size is about 40k bytes, so this loops about 5 times
                a = bytes.find(b'\xff\xd8')
                b = bytes.find(b'\xff\xd9')
                if a>b:
                    bytes = bytes[b+2:]
                    continue
                if a!=-1 and b!=-1:
                    jpg = bytes[a:b+2]
                    #bytes= bytes[b+2:]
                    #print("found image", a, b, len(bytes))
                    break
            img = cv2.imdecode(numpy.frombuffer(jpg, dtype=numpy.uint8),cv2.IMREAD_COLOR)
            # Resize to half size so that image processing is faster
            img = cv2.resize(img, ((int)(len(img[0])/RESIZE_SCALE),(int)(len(img)/RESIZE_SCALE)))
            
            with imageLock:
                self.latestImg = copy.deepcopy(img) # Make a copy not a reference

            masked = self.doImgProc() #pass by reference for all non-primitve types in Python

            # after image processing you can update here to see the new version
            with imageLock:
                self.feedback = copy.deepcopy(masked)

    def setThresh(self, name, value):
        self.thresholds[name] = value
    
    def doImgProc(self):
        """
        low = (self.thresholds['low_blue'], self.thresholds['low_green'], self.thresholds['low_red'])
        high = (self.thresholds['high_blue'], self.thresholds['high_green'], self.thresholds['high_red'])
        theMask = cv2.inRange(self.latestImg, low, high)
        """
        
        # Lab 3
        
        low = (self.thresholds['lo_hue'], self.thresholds['lo_saturation'], self.thresholds['lo_value'])
        high = (self.thresholds['hi_hue'], self.thresholds['hi_saturation'], self.thresholds['hi_value'])
        

        """
        # defined low and high values for beachball
        low = (self.yellowBeachball['lo_hue'], self.yellowBeachball['lo_saturation'], self.yellowBeachball['lo_value'])
        high = (self.yellowBeachball['hi_hue'], self.yellowBeachball['hi_saturation'], self.yellowBeachball['hi_value'])
        """

        """
        cv2.cvtColor(self.latestImg, cv2.COLOR_RGB2HSV_FULL)    # convert to HSV
        theMask = cv2.inRange(self.latestImg, low, high)    # mask image
        """

        """
        # erode and dilate to remove noise
        kernel = numpy.ones((3, 3), numpy.uint8)
        kernel2 = numpy.ones((5, 5), numpy.uint8)
        theMask = cv2.erode(theMask, kernel2, iterations=4)
        theMask = cv2.dilate(theMask, kernel, iterations=5)
        theMask = cv2.erode(theMask, kernel, iterations=4)        

        components, labels, stats, centroids = cv2.connectedComponentsWithStats(theMask, connectivity=8, ltype=cv2.CV_32S)

        # self.drawCircle(stats) # draw circle ontop of image
        
        self.objCentroid = self.findCenter(stats) # find center of object
        """
        
        # Lab 4

        # cones
        greenLo = (self.greenCone['lo_hue'], self.greenCone['lo_saturation'], self.greenCone['lo_value'])
        greenHi = (self.greenCone['hi_hue'], self.greenCone['hi_saturation'], self.greenCone['hi_value'])

        redLo = (self.redCone['lo_hue'], self.redCone['lo_saturation'], self.redCone['lo_value'])
        redHi = (self.redCone['hi_hue'], self.redCone['hi_saturation'], self.redCone['hi_value'])

        kernel = numpy.ones((3,3), numpy.uint8)

        greenMask = cv2.inRange(self.latestImg, greenLo, greenHi)
        redMask = cv2.inRange(self.latestImg, redLo, redHi)

        greenMask = cv2.erode(greenMask, kernel, iterations=2)
        greenMask = cv2.dilate(greenMask, kernel, iterations=4)

        redMask = cv2.erode(redMask, kernel, iterations=2)
        redMask = cv2.dilate(redMask, kernel, iterations=4)

        # beachball
        low = (self.yellowBeachball['lo_hue'], self.yellowBeachball['lo_saturation'], self.yellowBeachball['lo_value'])
        high = (self.yellowBeachball['hi_hue'], self.yellowBeachball['hi_saturation'], self.yellowBeachball['hi_value'])

        yellowMask = cv2.inRange(self.latestImg, low, high)

        kernel2 = numpy.ones((3, 3), numpy.uint8)
        kernel3 = numpy.ones((5, 5), numpy.uint8)
        yellowMask = cv2.erode(yellowMask, kernel2, iterations=4)
        yellowMask = cv2.dilate(yellowMask, kernel3, iterations=5)
        yellowMask = cv2.erode(yellowMask, kernel3, iterations=4)  


        theMask = cv2.bitwise_or(greenMask, redMask)
        theMask = cv2.bitwise_or(theMask, yellowMask)

        components, labels, greenStats, centroid = cv2.connectedComponentsWithStats(greenMask, connectivity=8, ltype=cv2.CV_32S)
        components, labels, redStats, centroid = cv2.connectedComponentsWithStats(redMask, connectivity=8, ltype=cv2.CV_32S)
        components, labels, yellowStats, centroid = cv2.connectedComponentsWithStats(yellowMask, connectivity=8, ltype=cv2.CV_32S)

        self.greenCenter = self.findCenter(greenStats)
        self.redCenter = self.findCenter(redStats)
        self.objCentroid = self.findCenter(yellowStats)

        print(self.greenCenter)

        # END TODO
        return cv2.bitwise_and(self.latestImg, self.latestImg, mask=theMask)
    
    def drawCircle(self, statsArr):
        # extract the 4th element from each row and sort
        pxCount = [row[3] for row in statsArr]
        pxSorted = sorted(pxCount, reverse=True)

        if len(pxSorted) < 2:
            return      # no object in image

        maskTarget = pxSorted[1] # target object

        # find the index of the target object
        objIndx = [i for i, row in enumerate(statsArr) if row[3] == maskTarget]
        if not objIndx:
            return  # safety check

        idx = objIndx[0]
        # draw circle onto image
        
        self.latestImg = cv2.circle(self.latestImg,(int(statsArr[idx][cv2.CC_STAT_LEFT] + statsArr[idx][cv2.CC_STAT_WIDTH] / 2),
                                                    int(statsArr[idx][cv2.CC_STAT_TOP] + statsArr[idx][cv2.CC_STAT_HEIGHT] / 2)),
                                                    50,(360, 255, 255),2)

        return
    
    def findCenter(self, statsArr):
        # extract the 4th element from each row and sort
        pxCount = [row[3] for row in statsArr]
        pxSorted = sorted(pxCount, reverse=True)

        if len(pxSorted) < 2:
            self.visible = False
            return      # no object in image

        maskTarget = pxSorted[1] # target object

        # find the index of the target object
        objIndx = [i for i, row in enumerate(statsArr) if row[3] == maskTarget]
        if not objIndx:
            self.visible = False
            return  # safety check

        idx = objIndx[0]

        # draw circle onto image
        self.latestImg = cv2.circle(self.latestImg,(int(statsArr[idx][cv2.CC_STAT_LEFT] + statsArr[idx][cv2.CC_STAT_WIDTH] / 2),
                                                    int(statsArr[idx][cv2.CC_STAT_TOP] + statsArr[idx][cv2.CC_STAT_HEIGHT] / 2)),
                                                    10,(360, 255, 255), 3)

        self.visible = True

        # find center
        return (int(statsArr[idx][cv2.CC_STAT_LEFT] + statsArr[idx][cv2.CC_STAT_WIDTH] / 2),
                    int(statsArr[idx][cv2.CC_STAT_TOP] + statsArr[idx][cv2.CC_STAT_HEIGHT] / 2))


# END OF IMAGEPROC


if __name__ == "__main__":
    
    cv2.namedWindow("Create View", flags=cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow("Create View", 21, 21)
    
    cv2.namedWindow('sliders')
    cv2.moveWindow('sliders', 680, 21)
    
    sm = StateMachine()
    sm.start()
    
    # Probably safer to do this on the main thread rather than in ImgProc init
    """
    cv2.createTrackbar('low_red', 'sliders', sm.video.thresholds['low_red'], 255,
                      lambda x: sm.video.setThresh('low_red', x) )
    cv2.createTrackbar('high_red', 'sliders', sm.video.thresholds['high_red'], 255,
                     lambda x: sm.video.setThresh('high_red', x) )
    
    cv2.createTrackbar('low_green', 'sliders', sm.video.thresholds['low_green'], 255,
                      lambda x: sm.video.setThresh('low_green', x) )
    cv2.createTrackbar('high_green', 'sliders', sm.video.thresholds['high_green'], 255,
                     lambda x: sm.video.setThresh('high_green', x) )
    
    cv2.createTrackbar('low_blue', 'sliders', sm.video.thresholds['low_blue'], 255,
                      lambda x: sm.video.setThresh('low_blue', x) )
    cv2.createTrackbar('high_blue', 'sliders', sm.video.thresholds['high_blue'], 255,
                     lambda x: sm.video.setThresh('high_blue', x) )
    """

    # HSV sliders
    cv2.createTrackbar('lo_hue', 'sliders', sm.video.thresholds['lo_hue'], 360,
                      lambda x: sm.video.setThresh('lo_hue', x) )
    cv2.createTrackbar('hi_hue', 'sliders', sm.video.thresholds['hi_hue'], 360,
                     lambda x: sm.video.setThresh('hi_hue', x) )
    
    cv2.createTrackbar('lo_saturation', 'sliders', sm.video.thresholds['lo_saturation'], 255,
                      lambda x: sm.video.setThresh('lo_saturation', x) )
    cv2.createTrackbar('hi_saturation', 'sliders', sm.video.thresholds['hi_saturation'], 255,
                     lambda x: sm.video.setThresh('hi_saturation', x) )
    
    cv2.createTrackbar('lo_value', 'sliders', sm.video.thresholds['lo_value'], 255,
                      lambda x: sm.video.setThresh('lo_value', x) )
    cv2.createTrackbar('hi_value', 'sliders', sm.video.thresholds['hi_value'], 255,
                     lambda x: sm.video.setThresh('hi_value', x) )

    while len(sm.video.latestImg) == 0 or len(sm.video.feedback) == 0:
        sleep(1)

    while(sm.RUNNING):
        with imageLock:
            cv2.imshow("Create View",sm.video.latestImg)
            cv2.imshow("sliders",sm.video.feedback)
        cv2.waitKey(5)

    cv2.destroyAllWindows()

    sleep(1)

    #sm.join()

