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
ALPHA = 0.15
EPSILON_HEADING = 0.5
EPSILON_DIST = 0.1
K_HEADING = 0.05
K_VEl = 5
vote_array = []

def obstacle(): 
    # CV, determine whether or not there is an obstacle there
    return False

def close(): 
    # ToF sensor, will decide whether we are close enough to AED
    return True

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
            car.look_for_cone()
            # print('cone position:', car.cone_position)
            if car.cone_position: 
                print('i see a cone!')
                car.avoid_cone()
                
                # pass
            if car.state == 0: ## go to AED waypoint #1
                car.target_x = 1.5
                car.target_y = 1.65
                car.go()
                if car.mini_state == 2:
                    car.mini_state = 0
                    car.state = 1
            elif car.state == 1: # go to AED waypoint #2
                car.target_x = 1
                car.target_y = 1.65
                car.go()
                if car.mini_state == 2:
                    car.mini_state = 0
                    car.state = 2
            elif car.state == 2: # go to AED 
                car.detect_april_tag()
                if car.mini_state == 1: 
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
                    car.state = 5
            elif car.state == 5: # turn around
                car.target_x = car.x + 0.5
                car.target_y = car.y -0.5
                car.go()
                if car.mini_state == 2:
                    car.state =6
                    car.mini_state =0
            elif car.state ==6: # go to april tag
                car.detect_april_tag()
                if car.mini_state == 1: 
                    car.state = 7
                    car.mini_state = 0
            elif car.state ==7: # dropoff aed
                car.stop()
                car.dropoffAED()
                print('Success! AED dropped off')
            car.prev_time = time.time()
            car.filter()
            car.sendArduino()
            #print('state' + str(car.state))
                
class Car(object): 
    def __init__(self): 
        self.cap = cv2.VideoCapture(0)
        self.detector = apriltag.Detector()
        
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

        self.state = 0
        self.prev_time = time.time()
        self.april_time = time.time()

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
    
    def straight(self, error): 
        val = error * K_VEl
        if val > STRAIGHT_VEL:
            self.leftVel, self.rightVel = -STRAIGHT_VEL, -STRAIGHT_VEL
        else: 
            self.leftVel, self.rightVel = -val, -val

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
            target_dx = self.target_x - self.x
            target_dy = self.target_y - self.y
            self.straight(math.sqrt(target_dx**2 + target_dy**2))
            if abs(target_dx) < EPSILON_DIST and abs(target_dy) < EPSILON_DIST:
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
            if largest_contour_area > 2500:
                # Calculate the center of the contour
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cone_detected = 1
                    print("cone detected")
                    

        # Display the frame with the cone position
        #cv2.imshow("Traffic Cone Detection", frame)
        # return
        #Voting to make it more robust
        
        vote_array.append(cone_detected)
        
        if len(vote_array) > 15:
            vote_array.pop(0)
        if sum(vote_array) > len(vote_array) / 2 and cx != -1 and cy != -1:
            #self.cone_position = (cx, cy)
            print("find cone")
            self.cone_position = (cx,cy)
          
        ''' 
        ret, frame = self.cap.read()
        if ret == True:
    # convert the image to HSV because easier to represent color in
    # HSV as opposed to in BGR 
            hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # define range of orange traffic cone color in HSV
            lower_orange1 = np.array([0, 135, 135])
            lower_orange2 = np.array([15, 255, 255])
            upper_orange1 = np.array([159, 135, 80])
            upper_orange2 = np.array([179, 255, 255])

            # threshold the HSV image to get only bright orange colors
            imgThreshLow = cv2.inRange(hsv_img, lower_orange1, lower_orange2)
            imgThreshHigh = cv2.inRange(hsv_img, upper_orange1, upper_orange2)

            # Bitwise-OR low and high threshes
            threshed_img = cv2.bitwise_or(imgThreshLow, imgThreshHigh)

            # smooth the image with erosion, dialation, and smooth gaussian
            # first create a kernel with standard size of 5x5 pixels
            kernel = np.ones((5,5),np.uint8)

            # get rid of small artifacts by eroding first and then dialating 
            threshed_img_smooth = cv2.erode(threshed_img, kernel, iterations = 3)
            threshed_img_smooth = cv2.dilate(threshed_img_smooth, kernel, iterations = 2)

            # account for cones with reflective tape by dialating first to bridge the gap between one orange edge
            # and another and then erode to bring the traffic cone back to standard size
            smoothed_img = cv2.dilate(threshed_img_smooth, kernel, iterations = 11)
            smoothed_img = cv2.erode(smoothed_img, kernel, iterations = 7)

            # detect all edges witin the image
            edges_img = cv2.Canny(smoothed_img, 100, 200)
            _,contours, hierarchy = cv2.findContours(edges_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # set parameters for writing text and drawing lines
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 2
            fontColor = (0, 0, 255)
            lineType = 2

            # analyze each contour and deterime if it is a triangle
            for cnt in contours:
                boundingRect = cv2.boundingRect(cnt)
                approx = cv2.approxPolyDP(cnt, 0.06 * cv2.arcLength(cnt, True), True)
                print(len(approx))
                # if the contour is a triangle, draw a bounding box around it and tag a traffic_cone label to it
                if len(approx) == 3:
                    print("cone_detected")
#                     x, y, w, h = cv2.boundingRect(approx)
#                     rect = (x, y, w, h)
#                     cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 3)
#                     bottomLeftCornerOfText = (x, y)
#                     cv2.putText(frame,'traffic_cone', 
#                         bottomLeftCornerOfText, 
#                         font, 
#                         fontScale,
#                         fontColor,
#                         lineType)

            # display the resulting frame
            cv2.imshow('Frame',frame)
        '''
        
    def avoid_cone(self):
        print('avoiding cone')
        #if(abs(self.cone_position[0] - self.MIDPOINT)< self.MIDPOINT/6):
        if (abs(self.cone_position[0] - self.SCREEN_WIDTH) < self.MIDPOINT/6) or (abs(self.cone_position[0] - 0) < self.MIDPOINT/6):
            self.straight()
        elif(self.cone_position[0] < self.MIDPOINT):
            self.leftVel = -STRAIGHT_VEL
            self.rightVel = -STRAIGHT_VEL/3
        else:
            self.leftVel = -STRAIGHT_VEL/3
            self.rightVel = -STRAIGHT_VEL
        #self.leftVel = -STRAIGHT_VEL/3
        #self.rightVel = -STRAIGHT_VEL 

    def detect_april_tag(self):    
        result, image = self.cap.read()
        grayimg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # look for tags
        detections = self.detector.detect(grayimg)
        target_dx = -0.1 - self.x
        target_dy = 1.65 - self.y
        print(target_dx, target_dy)
        vel = abs(target_dx) * K_VEl # P control velocity
        if vel> STRAIGHT_VEL/2: 
            vel = STRAIGHT_VEL/2
        if not detections:
            print("Nothing ")
            cone_position = None
            code_present = False
        else:
            code_present = True
            tag_position = detections[0].center
            # Get the corners of the AprilTag
            corners = detections[0].corners
            # Calculate the distance using triangulation
            pixel_width = abs(corners[0][0] - corners[1][0])
            dist_to_tag = 100.0/pixel_width
            #pixel size 200: roughly 30 cm
            print(tag_position)
            print("distance" + str(dist_to_tag))
        if code_present == False:
            print('no april tag')
        else:
            if self.mini_state == 0: 
                if dist_to_tag < 0.4: 
                    self.mini_state = 1
                else: 
                    fraction_diff = (self.MIDPOINT - tag_position[0])/self.MIDPOINT
                    self.leftVel= -vel - 3* fraction_diff
                    self.rightVel = -vel + 3* fraction_diff
            # elif self.mini_state ==1: 
            #     self.target_x = self.x - 0.05 ## set target to aed point
            #     self.target_y = self.y
            #     self.mini_state = 3

        # # detects whether there is an april tag in view
        # detector = Detector(families='tag36h11', 
        #                 nthreads=1,
        #                 quad_decimate=2,
        #                 quad_sigma=0.0,
        #                 refine_edges=1,
        #                 decode_sharpening=0.25,
        #                 debug=0,
        #                 ) #physical size of the apriltag
        # _, self.frame = self.cap.read()
        # self.frame = cv2.resize(self.frame, (640,480))
        # gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        # tags = detector.detect(gray, estimate_tag_pose=False, camera_params=self.intrisic, tag_size=self.tagsize)
        
        # if tags:
        #     tag = tags[0]
        #     print("TAG: " + str(tag))
        #     if tag.center[0] < 320 - 7:
        #         print("turn left")
        #         return 1   #turn left
        #     elif tag.center[0] > 320 + 7:
        #         print("turn right")
        #         return 2   #turn right
        #     else:
        #         return 3   # go straight

if __name__ == "__main__":
    main()
