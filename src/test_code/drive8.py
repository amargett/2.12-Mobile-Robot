import serial
import time
import numpy as np
import cv2
import math
import apriltag
import random as rng
# from pupil_apriltags import Detector

arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.1)
# arduino = serial.Serial(port='/dev/tty.usbserial-0264FEA5', baudrate=115200, timeout=.1)
orange_lower = np.array([1, 100, 150])
orange_upper = np.array([15, 200, 255])


STRAIGHT_VEL = 5
TURN_VEL = STRAIGHT_VEL/2
PICKUP_ANGLE = 40
DROPOFF_ANGLE = 120
ALPHA = 0.05
EPSILON_HEADING = 1
K_HEADING = 0.05
K_VEL_P = 8
K_CORR_P = 0.3
K_CORR_D = 0.008
Ki = 0.1

CAP = cv2.VideoCapture(0)
        
SCREEN_WIDTH = int(CAP.get(cv2.CAP_PROP_FRAME_WIDTH))
SCREEN_HEIGHT = int(CAP.get(cv2.CAP_PROP_FRAME_HEIGHT))
MIDPOINT = SCREEN_WIDTH // 2

P_CONTROL_BIAS = 0.25

FRAME_TIME = 2e-3

def obstacle(): 
    # CV, determine whether or not there is an obstacle there
    return False

def close(): 
    # ToF sensor, will decide whether we are close enough to AED
    return True

def main():
    car = Car()
    while [car.x0, car.y0, car.heading0] == [None, None, None]: # wait until readArduino receives usable data
        car.sendArduino()
        car.x0, car.y0, car.heading0 = car.readArduino()
    while True:
        if (time.time() - car.prev_time) > FRAME_TIME:
            car.prev_time = time.time()
            car.mega_counter += 1
            if car.mega_counter % 10 == 0:
                # print('MEGA' + str(car.mega_state))
                car.ret, car.frame = CAP.read()
                if car.state == 2:
                    car.detect_april_tag(0.35)
                elif car.state == 7:
                    car.detect_april_tag(0.6)
                else:
                    car.look_for_cone()
                # if car.state == 6:
                #     car.detect_april_tag(3 - car.x)
                # car.mega_state = 1
            else:
                # print('MEGA' + str(car.mega_state))
                car.readArduino()
                # car.mega_state = 0
                # continue looping until readArduino receives usable data
                if [car.x_raw, car.y_raw, car.heading_raw] != [None, None, None]: 
                    
                    car.setXYH()

                    if car.cone_position:
                        if car.prev_state is None:
                            car.avoid_cone()
                            car.prev_state = car.state
                        car.state = 10
                    elif car.state == 10:
                        car.go()
                        if car.mini_state == 2:
                            car.state = car.prev_state
                            car.prev_state = None
                    if car.state == 0: ## go to AED waypoint #1
                        car.target_x = 1.5
                        car.target_y = 1.65
                        car.go()
                        if car.mini_state == 2:
                            car.mini_state = 0
                            car.state = 1
                    elif car.state == 1: # go to AED waypoint #2
                        car.target_x = 0.75
                        car.target_y = 1.65
                        car.go()
                        if car.mini_state == 2:
                            car.mini_state = 0
                            car.state = 2
                    elif car.state == 2: # go to AED 
                        if car.mini_state == 2: 
                            car.state = 3
                            car.mini_state = 0
                    elif car.state == 3: # pickup AED
                        car.stop()
                        car.pickupAED()
                        print('Success! AED picked up')
                        car.pickup_counter += 1 
                        if car.pickup_counter > 200:
                            car.mini_state = 0
                            car.state = 4
                    elif car.state == 4: # back up
                        car.back()
                        car.backup_counter += 1
                        if car.backup_counter > 200:
                            car.mini_state = 0
                            car.state = 5
                    elif car.state == 5: # turn around
                        car.target_x = 2
                        car.target_y = 1
                        car.go()
                        if car.mini_state == 2: 
                            car.mini_state = 0
                            car.state = 7
                            # car.stop()
                    # elif car.state == 6: # go forward
                    #     car.target_x = 2.75
                    #     car.target_y = 1
                    #     car.go()
                    #     if car.mini_state == 2: 
                    #         car.mini_state = 0
                    #         car.state = 7
                    #         # car.stop()
                    elif car.state == 7: # go to april tag
                        if car.mini_state == 2: 
                            car.mini_state = 0
                            car.state = 8
                    elif car.state == 8: # dropoff aed
                        car.stop()
                        car.dropoffAED()
                        print('Success! AED dropped off')
                    
                car.filter()
                car.sendArduino()
                car.printCurr()
                # print('state' + str(car.state))
                
class Car(object): 
    def __init__(self): 
        self.detector = apriltag.Detector()
        self.ret = None
        self.frame = None
        self.mega_counter = 0

        self.epsilon_dist = 0.1
        
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

        self.mega_state = 0
        self.state = 0
        self.prev_time = time.time()
        self.april_time = time.time()
        self.prev_state = None

        self.pickup_counter = 0
        self.backup_counter = 0
        self.dropoff_counter = 0

        self.mini_state = 0
        self.cone_position = None
        self.april_tag = None
        self.tagangle = 0

        self.frame = None
        self.intrisic = [640,640,960,540]
        self.tagsize = 0.100  #physical size of printed tag, unit = meter
        self.threshold = 14  # tolerable yaw
        self.vote_array = []

        self.sumerror = 0
        self.prev_error_heading = 0

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
    
    def straight(self, error_dist, error_heading): 
        val = K_VEL_P*error_dist
        delta_val = K_CORR_P*error_heading + K_CORR_D*(error_heading - self.prev_error_heading)/FRAME_TIME
        # self.leftVel, self.rightVel = min(-val - delta_val, -STRAIGHT_VEL), min(-val + delta_val, -STRAIGHT_VEL)

        if val > STRAIGHT_VEL:
            self.leftVel, self.rightVel = -STRAIGHT_VEL - delta_val, -STRAIGHT_VEL + delta_val
        else: 
            self.leftVel, self.rightVel = -val - delta_val, -val + delta_val
        self.prev_error_heading = error_heading

    def left(self, error): 
        self.leftVel, self.rightVel = -K_HEADING*error - P_CONTROL_BIAS, K_HEADING*error + P_CONTROL_BIAS
        if self.rightVel > 2.5: 
            self.leftVel, self.rightVel = -2.5, 2.5

    def right(self, error): 
        self.leftVel, self.rightVel = K_HEADING*error + P_CONTROL_BIAS, -K_HEADING*error - P_CONTROL_BIAS
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
        print('left vel: %f, right vel %f' % (self.filtLeftVel, self.filtRightVel))
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
        target_dx = self.target_x - self.x
        target_dy = self.target_y - self.y
        self.target_dheading = 360 - (np.degrees(np.arctan2(target_dy, target_dx)) + 360) % 360
        error_heading = self.heading - self.target_dheading
        
        if self.mini_state == 0: # rotate
            print(self.mini_state, self.heading, self.target_dheading)
            if abs(error_heading) < EPSILON_HEADING:
                self.mini_state = 1
            elif self.target_dheading > 180: 
                self.left(abs(error_heading))
            else: 
                self.right(abs(error_heading))
        elif self.mini_state == 1: # go straight
            target_dx = self.target_x - self.x
            target_dy = self.target_y - self.y
            self.straight(math.sqrt(target_dx**2 + target_dy**2), error_heading)
            if abs(target_dx) < self.epsilon_dist and abs(target_dy) < self.epsilon_dist:
                print("You did it!")
                print("target x: " + str(self.target_x) + ' target y: ' + str(self.target_y) + ' target dx: ' + str(target_dx) + ' target dy: ' + str(target_dy))
                self.mini_state = 2
        return self.mini_state
    
   
    def look_for_cone(self): 
        '''
        Read the frame from the video capture
        '''
        ret, frame = self.ret, self.frame
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
        _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Initialize the position of the cone
        self.cone_position = None
        cone_detected = 0
        state = 0
        cx = -1
        cy = -1
        #print("len contours =", len(contours))
        #print("x")
        #cv2.imshow("Masked Image", frame)
        # Process the contours
        if len(contours) > 0:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            # Calculate the area of the largest contour
            largest_contour_area = cv2.contourArea(largest_contour)
            if largest_contour_area > 10000:
                # Calculate the center of the contour
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cone_detected = 1
                    print("cone detected")

#                     if cx < midpoint and cx !=-1:
#                     print("cone on left")
#                     state = 1
#                     #left_desired_vel = -1
#                     #right_desired_vel = -3
                    
#                     elseif cx >= midpoint and cx != -1:
#                     print("cone on right")
#                     state = 2
#                     #left_desired_vel = -3
#                     #right_desired_vel = -1
                    

        # Display the frame with the cone position
        #cv2.imshow("Traffic Cone Detection", frame)
        # return
        #Voting to make it more robust
        
        self.vote_array.append(cone_detected)
        
        if len(self.vote_array) > 5:
            self.vote_array.pop(0)
        if sum(self.vote_array) > len(self.vote_array) / 2 and cx != -1 and cy != -1:
            #self.cone_position = (cx, cy)
            print("find cone")
            self.cone_position = (cx,cy)
        
    def avoid_cone(self):
        print('avoiding cone')
        # if(abs(self.cone_position[0] - MIDPOINT)< MIDPOINT/6):
        left = False
        right = False
        if(self.cone_position[0] < MIDPOINT):
            print("turning right")
            right = True
        else:
            print("turning left")
            left = True
        if right: 
            # self.old_target_x = self.target_x
            # self.old_target_y = self.target_y
            phi = 360 - self.heading
            self.target_x = self.x + 0.5*math.cos(math.radians(phi+45))
            self.target_y = self.y + 0.5*math.sin(math.radians(phi+45))
            pass
        elif left: 
            # self.old_target_x = self.target_x
            # self.old_target_y = self.target_y
            self.target_x = self.x + 0.5*math.sin(math.radians(phi-45))
            self.target_y = self.y + 0.5*math.cos(math.radians(phi-45))
        print("left vel", self.leftVel)
        print("right vel", self.rightVel)
        #self.leftVel = -STRAIGHT_VEL/3
        #self.rightVel = -STRAIGHT_VEL 


    def detect_april_tag(self, dist): 
        print('detecting tag')   
        image = self.frame
        grayimg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # look for tags
        detections = self.detector.detect(grayimg)
        if not detections:
            print("Nothing ")
            code_present = False
        else:
            code_present = True
            tag_position = detections[0].center
            # Get the corners of the AprilTag
            corners = detections[0].corners
            # Calculate the distance using triangulation
            pixel_width = abs(corners[0][0] - corners[1][0])
            dist_to_tag = 100.0/pixel_width
            vel = min(dist_to_tag * K_VEL_P, STRAIGHT_VEL) # P control velocity
            #pixel size 200: roughly 30 cm
            print(tag_position)
            print("distance" + str(dist_to_tag))
        if code_present == False:
            print('no april tag')
            self.leftVel = 1
            self.rightVel = -1

        else:
            if self.mini_state == 0: 
                if dist_to_tag < dist: 
                    self.mini_state = 2
                else: 
                    fraction_diff = (MIDPOINT - tag_position[0])/MIDPOINT
                    self.leftVel= -vel - 3* fraction_diff
                    self.rightVel = -vel + 3* fraction_diff

if __name__ == "__main__":
    main()
