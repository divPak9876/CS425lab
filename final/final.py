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

IP_ADDRESS = "192.168.1.105" 	# SET THIS TO THE RASPBERRY PI's IP ADDRESS
RESIZE_SCALE = 2 # try a larger value if your computer is running slow.
ENABLE_ROBOT_CONNECTION = False

# You should fill this in with your states
class States(enum.Enum):
    FOLLOW = enum.auto()
    LOST = enum.auto()

class StateMachine(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)   # MUST call this to make sure we setup the thread correctly
        # CONFIGURATION PARAMETERS
        global IP_ADDRESS
        self.IP_ADDRESS = IP_ADDRESS
        self.CONTROLLER_PORT = 5001
        self.TIMEOUT = 10					# If its unable to connect after 10 seconds, give up.  Want this to be a while so robot can init.
        self.STATE = States.FOLLOW
        self.RUNNING = True
        self.DIST = False
        self.video = ImageProc()

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

            """"
            Process here will find path for Ferb using image data and maybe sensor data?
            """

            # line following state
            if self.STATE == States.FOLLOW:
                # fetch centroids
                cx_top = self.video.centroids(0)
                cx_bottom = self.video.centroids(1)

                # start control loop
                if cx_bottom is None:
                   with socketLock:
                    self.sock.sendall("a drive_direct(0, 0)".encode())
                    self.sock.recv(128)
                   self.STATE = States.LOST

                # error
                center = self.screenW // 2
                lateral_error = cx_bottom - center

                # heading error
                if cx_top is not None:
                    angle_error = cx_top - cx_bottom
                else:
                    angle_error = 0  # not available

                # gains
                Kp_lat = 0.004     # lateral centering
                Kp_ang = 0.002     # predictive turning
                
                base_speed = 100

                # control output
                steering = Kp_lat * lateral_error + Kp_ang * angle_error

                left  = int(base_speed + steering)
                right = int(base_speed - steering)

                cmd = f"a drive_direct({left}, {right})"

                with socketLock:
                    self.sock.sendall(cmd.encode())
                    self.sock.recv(128)

            # quitting
            elif self.STATE == States.LOST:
                print("Lost line...giving up :(")
                self.RUNNING = False


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
        self.thresholds = {'lo_hue':31,'lo_saturation':47,'lo_value':150,'hi_hue':80,'hi_saturation':94,'hi_value':183}
        
        self.centroids = None

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
        low = (self.thresholds['lo_hue'], self.thresholds['lo_saturation'], self.thresholds['lo_value'])
        high = (self.thresholds['hi_hue'], self.thresholds['hi_saturation'], self.thresholds['hi_value'])

        lineMask = cv2.inRange(self.latestImg, low, high)

        # erosion and dilation
        kernel = numpy.ones((3,3), numpy.uint8)
        lineMask = cv2.erode(lineMask, kernel, iterations=1)
        lineMask = cv2.dilate(lineMask, kernel, iterations=3)
        lineMask = cv2.erode(lineMask, kernel, iterations=2)

        # find centroids and update screen
        self.centroids, lineMask = self.get_centroids(lineMask)

        # END TODO
        return cv2.bitwise_and(self.latestImg, self.latestImg, mask=lineMask)
    
    def centroid_x(slice):
            M = cv2.moments(slice)
            if M["m00"] > 0:
                return int(M["m10"] / M["m00"])
            else:
                return None

    def get_centroids(self, mask):

        h, w = mask.shape

        # Slice boundaries
        top_y1, top_y2 = int(h*0.50), int(h*0.75)
        bot_y1, bot_y2 = int(h*0.75), h

        # Extract slices
        top_slice = mask[top_y1:top_y2, :]
        bottom_slice = mask[bot_y1:bot_y2, :]

        cx_top = self.centroid_x(top_slice)
        cx_bottom = self.centroid_x(bottom_slice)

        # Centroid y-positions (middle of each slice)
        cy_top = (top_y1 + top_y2) // 2
        cy_bottom = (bot_y1 + bot_y2) // 2

        pts = []

        # Collect valid points
        pts.append((cx_top, cy_top) if cx_top    is not None else None)
        pts.append((cx_bottom, cy_bottom) if cx_bottom is not None else None)

        # Draw centroid points
        for p in pts:
            if p is not None:
                cv2.circle(mask, p, 6, (0, 255, 255), -1)

        # Draw connecting lines
        if pts[0] is not None and pts[1] is not None:
            cv2.line(mask, pts[0], pts[1], (0, 255, 255), 2)

        return (cx_top, cx_bottom), mask


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