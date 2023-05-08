import serial
import time
import cv2
import numpy as np
from pupil_apriltags import Detector
import argparse
import shutil
import threading
from detect import detectobstacle
from detect import motor_control
from pathlib import Path
from sys import platform

from models import *
from utils.datasets import *
from utils.utils import *

arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.1) # bradyn
# arduino = serial.Serial(port='/dev/tty.usbserial-0264FEA5', baudrate=115200, timeout=.1) # josh
# arduino = serial.Serial(port='/dev/cu.usbserial-0264FEA5', baudrate=115200, timeout=.1) # ashley

# if arduino.is_open(): 
#     print("port is open")
# else: 
#     print("port is closed")
    
STRAIGHT_VEL = 5
TURN_VEL = STRAIGHT_VEL/2
pickup_angle = 40
dropoff_angle = 120
target_x1 = 1
target_y1 = 1.65
alpha = 0.15
epsilon_heading = 2

def readArduino():
    '''
    Read output from Arduino: x, y, heading.
    Returns [None, None, None] if response is not in the right format
    '''
    if arduino.in_waiting > 0:
        try:
            response = arduino.readline().decode().strip().split(',')
            if len(response) == 3 and all(len(i) > 0 for i in response):
                response = [float(i) for i in response]
                return response
        except:
            pass
    return [None, None, None]

def sendArduino(left_velocity, right_velocity, servo_angle):
    # if arduino.in_waiting > 0:
    # print(left_velocity, right_velocity, servo_angle)
    msg = f"{left_velocity},{right_velocity},{servo_angle}\n".encode() # encode message as bytes
    arduino.write(msg)
    
def go_straight(leftVel, rightVel, servoAngle):
    leftVel = -STRAIGHT_VEL
    rightVel = -STRAIGHT_VEL
    return leftVel, rightVel, servoAngle

def go_back(leftVel, rightVel, servoAngle):
    leftVel = STRAIGHT_VEL
    rightVel = STRAIGHT_VEL
    return leftVel, rightVel, servoAngle

def turn_left(leftVel, rightVel, servoAngle):
    leftVel = -TURN_VEL
    rightVel = TURN_VEL
    return leftVel, rightVel, servoAngle

def turn_right(leftVel, rightVel, servoAngle):
    leftVel = TURN_VEL
    rightVel = -TURN_VEL
    return leftVel, rightVel, servoAngle

def stop(leftVel, rightVel, servoAngle):
    leftVel = 0
    rightVel = 0
    return leftVel, rightVel, servoAngle

def pickup_aed(leftVel, rightVel, servoAngle):
    servoAngle = pickup_angle
    return leftVel, rightVel, servoAngle

def dropoff_aed(leftVel, rightVel, servoAngle):
    servoAngle = dropoff_angle
    return leftVel, rightVel, servoAngle

def detect_apriltag(frame):

    detector = Detector(families='tag36h11', 
                    nthreads=1,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0,
                    ) 

    intrisic = [820,814,320,240] # camera parameters, [fx, fy cx, cy]
    tagsize = 0.100  #physical size of printed tag, unit = meter

    frame = cv2.resize(frame, (640,480))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray, estimate_tag_pose=True, camera_params=intrisic, tag_size=tagsize)
    
    if not tags:
        return 0  #nothing
    else:
        for tag in tags:
            #print(tag)
            center = [320,240]
            #frame = plotPoint(frame, tag.center, CENTER_COLOR)
            #frame = plotPoint(frame, center, CENTER_COLOR)
            #frame = plotText(frame, tag.center, CENTER_COLOR, tag.pose_R)
            #for corner in tag.corners:
            #    frame = plotPoint(frame, corner, CORNER_COLOR)
            #print(frame.shape)
            if tag.center[0] < 320 - 5:
                return 1   #turn left
            elif tag.center[0] > 320 + 5:
                return 2   #turn right
            else:
                return 3   # go straight

    # Return the detections

def obstacle(): 
    # CV, determine whether or not there is an obstacle there
    return True

def close(): 
    # ToF sensor, will decide whether we are close enough to AED
    return True

def main():
    x0, y0, heading0 = [None, None, None]
    while [x0, y0, heading0] == [None, None, None]: # wait until readArduino receives usable data
        x0, y0, heading0 = readArduino()
    x, y, heading = [0, 0, 0]
    dx, dy, dheading = [0, 0, 0]
    leftVel, rightVel, servoAngle = [0, 0, 90]
    filtLeftVel, filtRightVel, filtServoAngle = [0, 0, 90]
    state = 0
    success = False
    prev_time = time.time()
    pickup_counter = 0
    dropoff_counter = 0

    # ##Apriltag detection
    # cap=cv2.VideoCapture(0)  #camera used
    # looping = True
    # while looping:
    #     ret, frame = cap.read()
    #     state = detect_apriltag(frame)
    #     if state == 0:
    #         print("NOTHING")
    #     elif state == 1:
    #         print("TURN LEFT")
    #     elif state == 2:
    #         print("TURN RIGHT")
    #     elif state == 3:
    #         print("GO STRAIGHT")

    #     key = cv2.waitKey(1000) #ms
	# # terminate the loop if the 'Return' key is hit
    #     if key == 13:
    #         looping = False

    # cv2.destroyAllWindows()

    # #####
## obstacle detection	
global obstacle

    parser = argparse.ArgumentParser()
    parser.add_argument('--cfg', type=str, default='cfg/yolov3.cfg', help='cfg file path')
    parser.add_argument('--weights', type=str, default='weights/best.pt', help='path to weights file')
    parser.add_argument('--images', type=str, default='data/samples', help='path to images')
    parser.add_argument('--img-size', type=int, default=32 * 13, help='size of each image dimension')
    parser.add_argument('--conf-thres', type=float, default=0.50, help='object confidence threshold')
    parser.add_argument('--nms-thres', type=float, default=0.45, help='iou threshold for non-maximum suppression')
    opt = parser.parse_args()
    print(opt)
    def callback():
        global obstacle_detected
        #print("Obstacle detected: ", obstacle_detected)
    obstacle_detection_thread = threading.Thread(target=detectobstacle, args=(opt.cfg, opt.weights, opt.images), kwargs={"img_size":opt.img_size, "conf_thres":opt.conf_thres, "nms_thres":opt.nms_thres, "callback":callback})
    obstacle_detection_thread.start()
    

##obstacle detection

    while True:
        x, y, heading = readArduino()
        # continue looping until readArduino receives usable data and at least 5 milliseconds have passed
        if [x, y, heading] != [None, None, None] and (time.time() - prev_time) > 5e-3: 
            dx = x0 - x
            dy = y - y0
            if state == 0: # go straight
                leftVel, rightVel, servoAngle = go_straight(leftVel, rightVel, servoAngle)
                if dx >= target_x1: # reached target x
                    state = 1
            elif state == 1: # turn left
                leftVel, rightVel, servoAngle = turn_left(leftVel, rightVel, servoAngle)
                if abs(heading - 270) < epsilon_heading: # turned 90 degrees
                    state = 2
            elif state == 2: # go straight
                leftVel, rightVel, servoAngle = go_straight(leftVel, rightVel, servoAngle)
                if dy >= target_y1: # reached target y
                    state = 3
            elif state == 3: # turn left
                leftVel, rightVel, servoAngle = turn_left(leftVel, rightVel, servoAngle)
                if abs(heading - 180) < epsilon_heading: # turned 180 degrees
                    state = 4
            elif state == 4: # go straight
                leftVel, rightVel, servoAngle = go_straight(leftVel, rightVel, servoAngle)
                if dx <= -0.1:
                    leftVel, rightVel, servoAngle = stop(leftVel, rightVel, servoAngle)
                    state = 5
            elif state == 5: # pick up aed
                leftVel, rightVel, servoAngle = pickup_aed(leftVel, rightVel, servoAngle)
                pickup_counter += 1 
                if pickup_counter > 200:
                    state = 6
            elif state == 6: # go back a little bit
                leftVel, rightVel, servoAngle = go_back(leftVel, rightVel, servoAngle)
                if dx >= 0.5:
                    state = 7
            elif state == 7: # turn backwards
                leftVel, rightVel, servoAngle = turn_left(leftVel, rightVel, servoAngle)
                if abs(heading - 20) < epsilon_heading:
                    state = 8
            elif state == 8: # go straight
                leftVel, rightVel, servoAngle = go_straight(leftVel, rightVel, servoAngle)
                if dx >= 2.9:
                    state = 9
            elif state == 9: # dropoff aed
                leftVel, rightVel, servoAngle = dropoff_aed(leftVel, rightVel, servoAngle)
                leftVel, rightVel, servoAngle = stop(leftVel, rightVel, servoAngle)
                dropoff_counter += 1
                if dropoff_counter > 200:
                    success = True

            prev_time = time.time()
            print('State: %f, x: %f, y: %f, heading: %f, servoAngle: %f' % (state, dx, dy, heading, servoAngle))
            if success is True:
                print('Success!')
            filtLeftVel = alpha*leftVel + (1 - alpha)*filtLeftVel
            filtRightVel = alpha*rightVel + (1 - alpha)*filtRightVel
            filtServoAngle = alpha*servoAngle + (1 - alpha)*filtServoAngle
            sendArduino(filtLeftVel, filtRightVel, filtServoAngle)
        else:
            pass
            
        
if __name__ == "__main__":
    main()
