import serial
import time
import numpy as np
import cv2
import math
from pupil_apriltags import Detector

arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.1)
# arduino = serial.Serial(port='/dev/tty.usbserial-0264FEA5', baudrate=115200, timeout=.1)

orange_lower = np.array([1, 100, 150])
orange_upper = np.array([15, 200, 255])

## global variables/ constants
STRAIGHT_VEL = 5
TURN_VEL = STRAIGHT_VEL/2
PICKUP_ANGLE = 40
DROPOFF_ANGLE = 120
ALPHA = 0.15
EPSILON_HEADING = 1
EPSILON_DIST = 0.1
K_HEADING = 0.05

def main():
    '''
    main loop
    '''
    car = Car()
    while [car.x0, car.y0, car.heading0] == [None, None, None]: # wait until readArduino receives usable data
        car.x0, car.y0, car.heading0 = car.readArduino()
    while True:
        car.readArduino()
        # continue looping until readArduino receives usable data and at least 5 milliseconds have passed
        if [car.x_raw, car.y_raw, car.heading_raw] != [None, None, None] and (time.time() - car.prev_time) > 1e-3: 
            car.setXYH()
            if (time.time() - car.obstacle_time) > 100e-3: # look for cones every 100 loops
                car.look_for_cone()
                if car.ob: # if object has been detected
                    if car.state != car.prev_state: # 1st time detecting obstacle
                        car.prev_state = car.state
                    car.state = 6
                if not car.ob and car.state == 6: # if object goes out of view
                    car.state = car.prev_state
                car.obstacle_time = time.time()
            if car.state == 0: # go to 1st waypoint, far from AED
                car.target_x = 1.5
                car.target_y = 1.65
                car.go()
                if car.mini_state == 2:
                    car.mini_state = 0
                    car.state = 1
            elif car.state == 1: # go to 2nd waypoint, pointed towards AED
                car.target_x = 1
                car.target_y = 1.65
                car.go()
                if car.mini_state == 2:
                    car.mini_state = 0
                    car.state = 2
            elif car.state == 3: # go straight and pick up AED
                car.target_x = -0.1
                car.target_y = 1.65
                car.go()
                if car.mini_state == 2: 
                    car.stop()
                    car.pickupAED()
                    print('Success! AED picked up')
                    car.pickup_counter += 1 
                    if car.pickup_counter > 200:
                        car.mini_state = 0
                        car.state = 3
            elif car.state == 4: # back up
                car.back()
                car.backup_counter += 1
                if car.backup_counter > 200:
                    car.state = 5
            elif car.state == 5: # turn around and go to UR5
                car.target_x = 3
                car.target_y = 1
                car.go()
                if car.mini_state == 2:
                    car.stop()
                    car.dropoffAED()
                    print('Success! AED dropped off')
                    car.mini_state = 0  
            elif car.state == 6: # avoid obstacle
                car.avoid_cone()
            car.prev_time = time.time()
            car.filter()
            car.printCurr()
            car.sendArduino()
                          
class Car(object):  
    def __init__(self): 
        self.cap = cv2.VideoCapture(0)
        
        self.SCREEN_WIDTH = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.SCREEN_HEIGHT = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.MIDPOINT = self.SCREEN_WIDTH // 2
        
        self.target_x = 1
        self.target_y = 1.65
        self.target_heading = 0
        
        self.leftVel = 0
        self.rightVel = 0
        self.servoAngle = 90

        self.x0 = None
        self.y0 = None
        self.heading0 = None

        self.x = 0
        self.y = 0
        self.heading = 0

        self.x_raw = 0
        self.y_raw = 0
        self.heading_raw = 0

        self.filtLeftVel = 0
        self.filtRightVel = 0
        self.filtServoAngle = 90

        self.prev_state = 0
        self.state = 0
        self.prev_time = time.time()
        self.obstacle_time = time.time()

        self.pickup_counter = 0
        self.backup_counter = 0
        self.dropoff_counter = 0

        self.mini_state = 0
        self.cone_position = None
        self.april_tag = None

        self.ob = None
        self.frame = None

    def readArduino(self):
        '''
        Read output from Arduino: x, y, heading.
        Returns [None, None, None] if response is not in the right format
        '''
        self.x_raw, self.y_raw, self.heading_raw = [None, None, None]
        if arduino.in_waiting > 0:
            try:
                response = arduino.readline().decode().strip().split(',')
                if len(response) == 3 and all(len(i) > 0 for i in response):
                    response = [float(i) for i in response]
                    self.x_raw, self.y_raw, self.heading_raw = response
            except:
                pass
        return self.x_raw, self.y_raw, self.heading_raw
    
    def straight(self): 
        self.leftVel, self.rightVel = -STRAIGHT_VEL, -STRAIGHT_VEL

    def left(self, error): 
        '''
        input: float error, the difference between desired angle and current angle
        runs P control on the turn velocity to slow down as it gets closer
        '''
        self.leftVel, self.rightVel = -K_HEADING*error, K_HEADING*error
        if self.rightVel > 2.5: 
            self.leftVel, self.rightVel = -2.5, 2.5

    def right(self, error): 
        '''
        input: float error, the difference between desired angle and current angle
        runs P control on the turn velocity to slow down as it gets closer
        '''
        self.leftVel, self.rightVel = K_HEADING*error, -K_HEADING*error
        if self.leftVel > 2.5: 
            self.leftVel, self.rightVel = 2.5, -2.5

    def stop(self):
        self.leftVel, self.rightVel = 0, 0
    
    def back(self): 
        self.leftVel, self.rightVel = STRAIGHT_VEL, STRAIGHT_VEL

    def pickupAED(self):
        self.servoAngle = PICKUP_ANGLE

    def dropoffAED(self):
        self.servoAngle = DROPOFF_ANGLE

    def setXYH(self): 
        '''
        sets x, y, and heading based on raw IMU data and initial values
        '''
        self.x = self.x0 - self.x_raw
        self.y = self.y_raw - self.y0
        self.heading = self.heading_raw

    def printCurr(self):
        print('State: %f, x: %f, y: %f, heading: %f, servoAngle: %f' % (self.state, self.x, self.y, self.heading, self.servoAngle))

    def filter(self): 
        '''
        applies low pass filter to velocity values
        '''
        self.filtLeftVel = ALPHA*self.leftVel + (1 - ALPHA)*self.filtLeftVel
        self.filtRightVel = ALPHA*self.rightVel + (1 - ALPHA)*self.filtRightVel
        self.filtServoAngle = ALPHA*self.servoAngle + (1 - ALPHA)*self.filtServoAngle

    def sendArduino(self): 
        '''
        sends filtered left & right velocity and desired servo angle
        '''
        msg = f"{self.filtLeftVel},{self.filtRightVel},{self.filtServoAngle}\n".encode() # encode message as bytes
        arduino.write(msg)
    
    def go(self): 
        '''
        Goes to target x,y pos based on current mini-state
        mini_state == 0: rotate
        mini_state == 1: go forward
        mini_state = 2: success!
        '''
        if self.mini_state == 0: # rotate
            target_dx = self.target_x - self.x
            target_dy = self.target_y - self.y
            self.target_dheading = 360 - (np.degrees(np.arctan2(target_dy, target_dx)) + 360) % 360
            print(self.mini_state, self.heading, self.target_dheading)
            if abs(self.heading - self.target_dheading) < EPSILON_HEADING:
                self.mini_state = 1
            elif self.target_dheading > 180: 
                self.left(abs(self.heading - self.target_dheading))
            else: 
                self.right(abs(self.heading - self.target_dheading))
        elif self.mini_state == 1: # go straight
            self.straight()
            if abs(self.x - self.target_x) < EPSILON_DIST and abs(self.y - self.target_y) < EPSILON_DIST:
                self.mini_state = 2
    
    def look_for_cone(self): 
        '''
        uses OpenCV to look for a cone
        updates self.ob
        ob = 0: obstacle on the right
        ob = 1: obstacle on the left
        '''
        # Read the frame from the video capture
        ret, self.frame = self.cap.read()
        if not ret:
            print("Failed to capture frame from camera")
            return
        # Convert the frame to the HSV color space
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        # Create a mask based on the orange color range
        mask = cv2.inRange(hsv, orange_lower, orange_upper)
        # Perform morphological operations to remove noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # Find contours in the mask
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Initialize the position of the cone
        self.cone_position = None
        # Process the contours
        if len(contours) > 0:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            # Calculate the center of the contour
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                self.cone_position = (cx, cy)
        if not self.cone_position:
            self.ob = None
        elif(self.cone_position[0] < self.MIDPOINT): #go right
            self.ob = 2
        else: #go left
            self.ob = 1
        
    def avoid_cone(self):
        '''
        curves left if object on right
        curves right if object on left
        '''
        if self.ob == 1: # go left
            self.leftVel = -STRAIGHT_VEL/3
            self.rightVel = -STRAIGHT_VEL
        elif self.ob == 2: # go right
            self.leftVel = -STRAIGHT_VEL
            self.rightVel = -STRAIGHT_VEL/3
        print('avoiding cone')

if __name__ == "__main__":
    main()