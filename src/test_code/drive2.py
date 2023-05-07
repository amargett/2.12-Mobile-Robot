import serial
import time
import cv2
import numpy as np
from pupil_apriltags import Detector

# arduino = serial.Serial(port='dev/TTYUSB0', baudrate=115200, timeout=.1)
arduino = serial.Serial(port='/dev/tty.usbserial-0264FEA5', baudrate=115200, timeout=.1)

VEL = 3
pickup_angle = 90
target_x = 1
target_y = 1
alpha = 0.2

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
    leftVel = -VEL
    rightVel = -VEL
    return leftVel, rightVel, servoAngle

def turn_left(leftVel, rightVel, servoAngle):
    leftVel = -VEL
    rightVel = VEL
    return leftVel, rightVel, servoAngle

def turn_right(leftVel, rightVel, servoAngle):
    leftVel = VEL
    rightVel = -VEL
    return leftVel, rightVel, servoAngle

def stop(leftVel, rightVel, servoAngle):
    leftVel = 0
    rightVel = 0
    return leftVel, rightVel, servoAngle

def pickup_aed(leftVel, rightVel, servoAngle):
    servoAngle = pickup_angle
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

    ##Apriltag detection
    cap=cv2.VideoCapture(0)  #camera used
    looping = True
    while looping:
        ret, frame = cap.read()
        state = detect_apriltag(frame)
        if state == 0:
            print("NOTHING")
        elif state == 1:
            print("TURN LEFT")
        elif state == 2:
            print("TURN RIGHT")
        elif state == 3:
            print("GO STRAIGHT")

        key = cv2.waitKey(1000) #ms
	# terminate the loop if the 'Return' key is hit
        if key == 13:
            looping = False

    cv2.destroyAllWindows()

    #####


    while True:
        x, y, heading = readArduino()
        # continue looping until readArduino receives usable data and at least 5 milliseconds have passed
        if [x, y, heading] != [None, None, None] and (time.time() - prev_time) > 5e-3: 
            dx = x0 - x
            dy = y0 - y
            dheading = heading - heading0
            if state == 0: # go straight
                leftVel, rightVel, servoAngle = go_straight(leftVel, rightVel, servoAngle)
                if dy >= target_y: # reached target y
                    state = 1
            elif state == 1: # turn left
                leftVel, rightVel, servoAngle = turn_left(leftVel, rightVel, servoAngle)
                if 180 <= dheading and dheading <= 270: # turned 90 degrees
                    state = 2
            elif state == 2: # go straight
                leftVel, rightVel, servoAngle = go_straight(leftVel, rightVel, servoAngle)
                if dx >= target_x: # reached target x
                    state = 3
            elif state == 3: # turn left
                leftVel, rightVel, servoAngle = turn_left(leftVel, rightVel, servoAngle)
                if 90 <= dheading and dheading <= 180: # turned 90 degrees
                    leftVel, rightVel, servoAngle = stop(leftVel, rightVel, servoAngle)
                    success = True

            prev_time = time.time()
            print(state, dx, dy, dheading)
            filtLeftVel = alpha*leftVel + (1 - alpha)*filtLeftVel
            filtRightVel = alpha*rightVel + (1 - alpha)*filtRightVel
            servoAngle = alpha*servoAngle + (1 - alpha)*filtServoAngle
            sendArduino(filtLeftVel, filtRightVel, filtServoAngle)
            if success is True:
                print('Success!')
                return
        else:
            pass
            
        
if __name__ == "__main__":
    main()