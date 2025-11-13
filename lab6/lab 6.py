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
    SEARCH = enum.auto()
    VISIBLE = enum.auto()
    TURN_L = enum.auto()
    TURN_R = enum.auto()
    FORWARD = enum.auto()
    FAR = enum.auto()
    MEDIUM = enum.auto()
    CLOSE = enum.auto()

class StateMachine(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)   # MUST call this to make sure we setup the thread correctly
        # CONFIGURATION PARAMETERS
        global IP_ADDRESS
        self.IP_ADDRESS = IP_ADDRESS
        self.CONTROLLER_PORT = 5001
        self.TIMEOUT = 10					# If its unable to connect after 10 seconds, give up.  Want this to be a while so robot can init.
        self.STATE = States.SEARCH
        self.RUNNING = True
        self.DIST = False
        self.video = ImageProc()
        self.leftScreen = 320 / 3
        self.rightScreen = self.leftScreen * 2
        self.topScreen = 2 * 240 / 5
        self.bottomScreen = 3 * 240 / 5

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
        self.cam = cv2.VideoCapture(0)

    def run(self):
        retValue, image = self.cam.read()

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

