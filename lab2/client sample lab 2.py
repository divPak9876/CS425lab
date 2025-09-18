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

socketLock = threading.Lock()

# You should fill this in with your states
class States(enum.Enum):
    DRIVE = enum.auto()
    TURN_L = enum.auto()
    TURN_R = enum.auto()

# Not a thread because this is the main thread which can be important for GUI access
class StateMachine():

    def __init__(self):
        # CONFIGURATION PARAMETERS
        self.IP_ADDRESS = "192.168.1.102" 	# SET THIS TO THE RASPBERRY PI's IP ADDRESS
        self.CONTROLLER_PORT = 5001
        self.TIMEOUT = 10					# If its unable to connect after 10 seconds, give up.  Want this to be a while so robot can init.
        self.STATE = States.DRIVE
        self.RUNNING = True
        self.DIST = False
        
        # connect to the motorcontroller
        try:
            with socketLock:
                self.sock = socket.create_connection( (self.IP_ADDRESS, self.CONTROLLER_PORT), self.TIMEOUT)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("Connected to RP")
        except Exception as e:
            print("ERROR with socket connection", e)
            sys.exit(0)
    
        # Collect events until released
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def main(self):
        # connect to the robot
        """ The i command will initialize the robot.  It enters the create into FULL mode which means it can drive off tables and over steps: be careful!"""
        with socketLock:
            self.sock.sendall("i /dev/ttyUSB0".encode())
            print("Sent command")
            result = self.sock.recv(128)
            print(result)
            if result.decode() != "i /dev/ttyUSB0":
                self.RUNNING = False
        
        self.sensors = Sensing(self.sock)
        # Start getting data
        self.sensors.start()

        # BEGINNING OF THE CONTROL LOOP
        while(self.RUNNING):
            sleep(0.1)
            if self.STATE == States.DRIVE:
                with socketLock:
                    self.sock.sendall("a drive_straight(50)".encode())
                    self.sock.recv(128)
                
                if self.sensors.frontLeftIR < 2000: 
                    self.STATE = States.TURN_L
                    with socketLock:
                        self.sock.sendall("a drive_straight(0)".encode())
                        self.sock.recv(128)
                    print("LEFT")
                    
                elif self.sensors.frontRightIR < 2000:
                    self.STATE = States.TURN_R
                    with socketLock:
                        self.sock.sendall("a drive_straight(0)".encode())
                        self.sock.recv(128)
                    print("RIGHT")

                """
                if self.sensors.frontLeftIR < 2000:
                    print("Line detected on front-right IR:", self.sensors.frontLeftIR)

                    # Load a one-note song
                    self.sock.sendall("a set_song(1, [[60,32]])".encode())
                    _ = self.sock.recv(128) 

                    # Play the song
                    self.sock.sendall("a play_song(1)".encode())
                    _ = self.sock.recv(128)  
                    print("Played one note (Middle C)")
                    sleep(1)
                """
            

        # END OF CONTROL LOOP
        
        # First stop any other threads talking to the robot
        self.sensors.RUNNING = False
        self.sensors.join()
        
        # Need to disconnect
        """ The c command stops the robot and disconnects.  The stop command will also reset the Create's mode to a battery safe PASSIVE.  It is very important to use this command!"""
        with socketLock:
            self.sock.sendall("c".encode())
            print(self.sock.recv(128))

        with socketLock:
            self.sock.close()
        # If the user didn't request to halt, we should stop listening anyways
        self.listener.stop()

    def on_press(self, key):
        # WARNING: DO NOT attempt to use the socket directly from here
        try:
            print('alphanumeric key {0} pressed'.format(key.char))
            if key.char == 'd':
                self.DIST = True
        except AttributeError:
            print('special key {0} pressed'.format(key))

    def on_release(self, key):
        # WARNING: DO NOT attempt to use the socket directly from here
        print('{0} released'.format(key))
        if key == keyboard.Key.esc or key == keyboard.Key.ctrl:
            # Stop listener
            self.RUNNING = False
            return False

# END OF STATEMACHINE


class Sensing(threading.Thread):
    def __init__(self, socket):
        threading.Thread.__init__(self)   # MUST call this to make sure we setup the thread correctly
        self.sock = socket
        self.RUNNING = True
        self.frontLeftIR = 3000
        self.frontRightIR = 3000
    
    def run(self):
        while self.RUNNING:
            # sleep(1)
            # This is where I would get a sensor update
            # Store it in this class
            # You can change the polling frequency to optimize performance, don't forget to use socketLock

            """
            with socketLock:
                # left IR
                self.sock.sendall("a cliff_left_signal".encode())
                self.leftIR = int(self.sock.recv(128).decode())
                # print("Left IR sensor value: ", self.leftIR)
            sleep(0.1)
            """

            with socketLock:
                # left front IR
                self.sock.sendall("a cliff_front_left_signal".encode())
                self.frontLeftIR = int(self.sock.recv(128).decode())
                # print("Front left IR sensor value: ", self.frontLeftIR)
            sleep(0.1)

            """
            with socketLock:
                # right IR
                self.sock.sendall("a cliff_right_signal".encode())
                self.rightIR = int(self.sock.recv(128).decode())
                # print("Right IR sensor value: ", self.rightIR)
            sleep(0.1)
            """

            with socketLock:  
                # right front IR
                self.sock.sendall("a cliff_front_right_signal".encode())
                self.frontRightIR = int(self.sock.recv(128).decode())
                # print("Front right IR sensor value: ", self.frontRightIR)
            sleep(0.1)         

            """
            with socketLock:
                self.sock.sendall("a battery_charge".encode())
                print("Battery charge: ", self.sock.recv(128).decode())
            """

                   

# END OF SENSING


if __name__ == "__main__":
    sm = StateMachine()
    sm.main()


