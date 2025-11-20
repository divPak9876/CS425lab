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
import math

socketLock = threading.Lock()
imageLock = threading.Lock()

IP_ADDRESS = "192.168.1.105" 	# SET THIS TO THE RASPBERRY PI's IP ADDRESS
RESIZE_SCALE = 2 # try a larger value if your computer is running slow.
ENABLE_ROBOT_CONNECTION = False

# You should fill this in with your states
class States(enum.Enum):
    LISTEN = enum.auto()
    INIT = enum.auto()
    GET_CIRCLE = enum.auto()


class StateMachine(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)   # MUST call this to make sure we setup the thread correctly
        # CONFIGURATION PARAMETERS
        global IP_ADDRESS
        self.IP_ADDRESS = IP_ADDRESS
        self.CONTROLLER_PORT = 5001
        self.TIMEOUT = 10					# If its unable to connect after 10 seconds, give up.  Want this to be a while so robot can init.
        self.STATE = States.INIT
        self.RUNNING = True
        self.DIST = False
        self.video = ImageProc()
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
        while(self.RUNNING):
            sleep(0.1)
            if self.STATE == States.INIT: # move a for error calc
                with socketLock:
                    self.sock.sendall("a drive_straight(50)".encode())
                    self.sock.recv(128)
                    sleep(0.5)
                    self.STATE = States.GET_CIRCLE # start chasing goal

            elif self.STATE == States.GET_CIRCLE:
                if self.video.distance == 0: # we are at goal, don't move
                    with socketLock:
                        self.sock.sendall("a drive_straight(0)".encode())
                        self.sock.recv(128)
                else:
                    # Gains (tune these)
                    Kp = 2.0
                    Kd = 0.5

                    ferb_head = self.video.heading
                    goal_head = self.video.headingGoal

                    if not ferb_head == None and not goal_head == None:
                        # compute error (wrap)
                        error = goal_head - ferb_head
                        error = (error + 180) % 360 - 180

                        # previous error handling
                        prev_error = getattr(self, "prev_error", 0.0)
                        d_error = error - prev_error
                        self.prev_error = error

                        # PD output
                        u = Kp*error + Kd*d_error

                        # clamp
                        u = max(min(u, 100), -100)

                        with socketLock:
                            # small deadband to avoid micro-wobble
                            if abs(error) < 2:
                                self.sock.sendall("a drive_straight(50)".encode())
                            else:
                                if u > 0:
                                    self.sock.sendall(f"a spin_left({int(abs(u))})".encode())
                                else:
                                    self.sock.sendall(f"a spin_right({int(abs(u))})".encode())

                            self.sock.recv(128)

                    """# error is diff between current heading and goal heading
                    ferb_head = self.video.heading
                    goal_head = self.video.headingGoal
                    error = ferb_head - goal_head

                    # turn until error 0
                    # if error angle is neg turn right, pos turn left
                    # idk where 0 is so I guessed for turning, feel free to change later - rowena
                    if error == 0:
                        with socketLock:                                    
                            self.sock.sendall("a drive_straight(50)".encode())
                            self.sock.recv(128)
                    elif error < 0: # turn right
                        with socketLock:                                    
                            self.sock.sendall("a spin_right(50)".encode())
                            self.sock.recv(128)
                    elif error > 0: # turn left
                        with socketLock:                                    
                            self.sock.sendall("a spin_left(50)".encode())
                            self.sock.recv(128)"""       
            elif self.STATE == States.LISTEN:
                pass

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
                self.sock.sendall("a distance".encode())
                print(self.sock.recv(128))


# END OF SENSING

class ImageProc(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)   # MUST call this to make sure we setup the thread correctly
        global IP_ADDRESS
        self.cam = cv2.VideoCapture(1)
        self.IP_ADDRESS = IP_ADDRESS
        self.PORT = 8081
        self.RUNNING = True
        self.latestImg = []
        self.feedback = []
        self.thresholds = {'lo_hue':0,'lo_saturation':121,'lo_value':42,'hi_hue':61,'hi_saturation':255,'hi_value':144}

        # positions
        self.goal = None
        self.lastPos = None
        self.currentPos = None
        
        # headings
        self.heading = None
        self.headingGoal = None
        self.distance = None


    def run(self):
        #url = "http://"+self.IP_ADDRESS+":"+str(self.PORT)
        #stream = urllib.request.urlopen(url)
        while(self.RUNNING):
            sleep(0.1)
            """bytes = b''
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
            img = cv2.imdecode(numpy.frombuffer(jpg, dtype=numpy.uint8),cv2.IMREAD_COLOR)"""
            retValue, img = self.cam.read()
            # Resize to half size so that image processing is faster
            img = cv2.resize(img, ((int)(len(img[0])/RESIZE_SCALE),(int)(len(img)/RESIZE_SCALE)))
            
            with imageLock:
                self.latestImg = copy.deepcopy(img) # Make a copy not a reference

            masked = self.doImgProc() #pass by reference for all non-primitve types in Python

            # after image processing you can update here to see the new version
            with imageLock:
                self.feedback = copy.deepcopy(masked)

    def click(self, event, x, y, flags, params):
        """
        - when user left clicks, store (x,y) target position
        - draw a circle at target position in image
        - move robot towards this circle i.e. target position
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            self.goal = (x, y)
            print("New goal:", self.goal)
            
    def setThresh(self, name, value):
        self.thresholds[name] = value
    
    def doImgProc(self):
        low = (self.thresholds['lo_hue'], self.thresholds['lo_saturation'], self.thresholds['lo_value'])
        high = (self.thresholds['hi_hue'], self.thresholds['hi_saturation'], self.thresholds['hi_value'])

        cv2.cvtColor(self.latestImg, cv2.COLOR_RGB2HSV_FULL)
        theMask = cv2.inRange(self.latestImg, low, high)

        kernel = numpy.ones((3,3), numpy.uint8)
        theMask = cv2.erode(theMask, kernel, iterations = 1)
        theMask = cv2.dilate(theMask, kernel, iterations = 5)
        theMask = cv2.erode(theMask, kernel, iterations = 1)

        # TODO: Work here
        if not self.goal == None:
            cv2.circle(self.latestImg, self.goal, 5, (0, 255, 0), 2)

        # update positions
        _,_,stats,_ = cv2.connectedComponentsWithStats(theMask, connectivity=8, ltype=cv2.CV_32S)
        self.lastPos = self.currentPos
        tempPos = self.findBoundingBox(stats)

        # compute ferb heading
        if not tempPos == None and not self.lastPos == None:
            x2, y2 = tempPos
            x1, y1 = self.lastPos

            if math.hypot(y2-y1, x2-x1) > 20:   # don't update position if the robot hasn't moved far
                self.heading = math.atan2(y2-y1, x2-x1)
                self.currentPos = tempPos

        # compute heading to goal and distance
        if not self.goal == None and not self.currentPos == None:
            x2, y1 = self.goal
            x1, y1 = self.currentPos
            self.headingGoal = math.atan2(y2-y1, x2-x1)
            cv2.line(self.latestImg, self.currentPos, self.goal, (0, 255, 255), 2)  # draw line between robot and goal

            self.distance = math.hypot(y2-y1, x2-x1)
            print(self.distance, " miles away from the goal.")
    
        # END TODO
        return cv2.bitwise_and(self.latestImg, self.latestImg, mask=theMask)
    
    def findBoundingBox(self, statsArr):
        # extract area from each row (usually row[4])
        areas = [row[4] for row in statsArr]
        areasSorted = sorted(areas, reverse=True)

        if len(areasSorted) < 2:
            self.visible = False
            return None  # no object

        targetArea = areasSorted[1]  # second largest object

        # find the index of the target object
        objIndx = [i for i, row in enumerate(statsArr) if row[4] == targetArea]
        if not objIndx:
            self.visible = False
            return None

        idx = objIndx[0]

        # bounding box
        left = statsArr[idx][cv2.CC_STAT_LEFT]
        top = statsArr[idx][cv2.CC_STAT_TOP]
        width = statsArr[idx][cv2.CC_STAT_WIDTH]
        height = statsArr[idx][cv2.CC_STAT_HEIGHT]

        self.visible = True

        # optional: draw rectangle on image
        self.latestImg = cv2.rectangle(self.latestImg, (left, top), (left + width, top + height), (0, 255, 255), 2)

        # return bounding box
        return (left + width/2, top + height/2)

# END OF IMAGEPROC


if __name__ == "__main__":
    
    cv2.namedWindow("Create View", flags=cv2.WINDOW_KEEPRATIO)
    cv2.moveWindow("Create View", 21, 21)
    
    cv2.namedWindow('sliders', flags=cv2.WINDOW_KEEPRATIO)
    cv2.moveWindow('sliders', 680, 21)
    
    sm = StateMachine()
    sm.start()
    
    cv2.setMouseCallback("Create View", sm.video.click, sm.video)
    
    # Probably safer to do this on the main thread rather than in ImgProc init
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

