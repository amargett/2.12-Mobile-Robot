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

STRAIGHT_VEL = 5
TURN_VEL = STRAIGHT_VEL/2
PICKUP_ANGLE = 40
DROPOFF_ANGLE = 120
ALPHA = 0.15
EPSILON_HEADING = 1
EPSILON_DIST = 0.1
K_HEADING = 0.05

def obstacle(): 
    # CV, determine whether or not there is an obstacle there
    return False

def close(): 
    # ToF sensor, will decide whether we are close enough to AED
    return True

# def turnaround(car):  
#     mini_state = 0
#     while True:
#         if mini_state == 0: 
#             car.back()
#             if abs(car.x - car.x0) >= target_x:
#                 mini_state =1
#         if mini_state == 1: 
#             car.left()
#             dheading = abs(car.heading - car.heading0)
#             if 29 < dheading < 31: 
#                 car.stop()
#                 return car
#         car.prev_time = time.time()
#         car.printCurr()
#         car.filter()
#         car.send()
def main():
    car = Car()
    while [car.x0, car.y0, car.heading0] == [None, None, None]: # wait until readArduino receives usable data
        car.x0, car.y0, car.heading0 = car.readArduino()
    while True:
        car.readArduino()
        # continue looping until readArduino receives usable data and at least 5 milliseconds have passed
        if [car.x_raw, car.y_raw, car.heading_raw] != [None, None, None] and (time.time() - car.prev_time) > 1e-3: 
            car.setXYH()
            # print(car.x, car.y, car.heading)
            # car.look_for_cone()
            # print('cone position:', car.cone_position)
            # if car.cone_position: 
            #     print('i see a cone!')
            #     car.avoid_cone()
                # pass
            if car.state == 0: ## go to AED waypoint
                car.target_x = 1.5
                car.target_y = 1.65
                car.go()
                if car.mini_state == 2:
                    car.mini_state = 0
                    car.state = 1
            elif car.state == 1:
                car.target_x = 1
                car.target_y = 1.65
                car.go()
                if car.mini_state == 2:
                    car.mini_state = 0
                    car.state = 2
            elif car.state == 2: # go to AED and pick it up
                if (time.time() - car.april_time) > 20e-3:
                    car.detect_april_tag()
                    car.april_time = time.time()
                if car.april_tag == 0: 
                    car.left(10)
                elif car.april_tag == 1: 
                    car.right(10)
                elif car.april_tag == 2: 
                    car.state = 3
            elif car.state == 3:
                car.target_x = -0.1
                car.target_y = 1.65
                car.straight()
                if abs(car.x - car.target_x) < 5*EPSILON_DIST and abs(car.y - car.target_y) < 5*EPSILON_DIST:
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
            elif car.state == 5: # turn around
                car.target_x = 3
                car.target_y = 1
                car.go()
                if car.mini_state == 2:
                    car.stop()
                    car.dropoffAED()
                    print('Success! AED dropped up')
                    car.state = 5
                    car.mini_state = 0
            elif car.state == 5:
                car.stop()
            car.prev_time = time.time()
            car.filter()
            car.printCurr()
            car.sendArduino()
                
class Car(object): 
    def __init__(self): 
        # self.cap = cv2.VideoCapture(0)
        
        # self.SCREEN_WIDTH = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        # self.SCREEN_HEIGHT = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # self.MIDPOINT = self.SCREEN_WIDTH // 2

        self.cap = None
        
        self.SCREEN_WIDTH = None
        self.SCREEN_HEIGHT = None
        self.MIDPOINT = None
        
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

        self.state = 0
        self.prev_time = time.time()
        self.april_time = time.time()

        self.pickup_counter = 0
        self.backup_counter = 0
        self.dropoff_counter = 0

        self.mini_state = 0
        self.cone_position = None
        self.april_tag = None

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
        self.leftVel, self.rightVel = -K_HEADING*error, K_HEADING*error
        if self.rightVel > 2.5: 
            self.leftVel, self.rightVel = -2.5, 2.5

    def right(self, error): 
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
        self.x = self.x0 - self.x_raw
        self.y = self.y_raw - self.y0
        self.heading = self.heading_raw

    def printCurr(self):
        print('State: %f, x: %f, y: %f, heading: %f, servoAngle: %f' % (self.state, self.x, self.y, self.heading, self.servoAngle))

    def filter(self): 
        self.filtLeftVel = ALPHA*self.leftVel + (1 - ALPHA)*self.filtLeftVel
        self.filtRightVel = ALPHA*self.rightVel + (1 - ALPHA)*self.filtRightVel
        self.filtServoAngle = ALPHA*self.servoAngle + (1 - ALPHA)*self.filtServoAngle

    def sendArduino(self): 
        msg = f"{self.filtLeftVel},{self.filtRightVel},{self.filtServoAngle}\n".encode() # encode message as bytes
        arduino.write(msg)
    
    def go(self): 
        '''
        Args:
            target_x(float)
            target_y(float)
            car(Car object)

        Returns:
            mini_state
        '''
        # mini_state == 0: rotate
        # mini_state == 1: go forward
        # mini_state = 2: success!
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
        return self.mini_state
    
    def look_for_cone(self): 
        # Read the frame from the video capture
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to capture frame from camera")
            return
        # Convert the frame to the HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Create a mask based on the orange color range
        mask = cv2.inRange(hsv, orange_lower, orange_upper)
        # Perform morphological operations to remove noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # Find contours in the mask
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
                # Draw a circle at the center of the contour
                # cv2.circle(frame, self.cone_position, 5, (0, 255, 0), -1)
        # Display the frame with the cone position
        # cv2.imshow("Traffic Cone Detection", frame)
        # return
    
    def avoid_cone(self):
        print('avoiding cone')
        if(abs(self.cone_position[0] - self.MIDPOINT)< self.MIDPOINT/6):
            self.straight()
        elif(self.cone_position[0] < self.MIDPOINT):
            self.leftVel = -STRAIGHT_VEL
            self.rightVel = -STRAIGHT_VEL/3
        else:
            self.leftVel = -STRAIGHT_VEL/3
            self.rightVel = -STRAIGHT_VEL

    def detect_april_tag(self): 
        # detects whether there is an april tag in view
        self.cap = cv2.VideoCapture(0)
        
        self.SCREEN_WIDTH = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.SCREEN_HEIGHT = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.MIDPOINT = self.SCREEN_WIDTH // 2

        detector = Detector(families='tag36h11', 
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0,
                        ) #physical size of the apriltag
        intrisic = [640,640,960,540]
        tagsize = 0.100  #physical size of printed tag, unit = meter
        threshold = 5  # tolerable yaw
        ret, self.frame = self.cap.read()
        self.frame = cv2.resize(self.frame, (640,480))
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray, estimate_tag_pose=False, camera_params=intrisic, tag_size=tagsize)
        if tags:
            for tag in tags:
                if tag.center[0] < 320 - threshold:
                    self.april_tag = 0 # turn left
                elif tag.center[0] > 320 + threshold:
                    self.april_tag = 1 # turn right
                else:
                    self.april_tag = 2 # go straight

if __name__ == "__main__":
    main()
