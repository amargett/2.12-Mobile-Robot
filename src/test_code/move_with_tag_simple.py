import serial
import time

import apriltag
import numpy as np
import cv2


# plot a little text
def plotText(image, center, color, text):
    center = (int(center[0]) + 4, int(center[1]) - 4)
    return cv2.putText(image, str(text), center, cv2.FONT_HERSHEY_SIMPLEX,
                       1, color, 3)

# setup and the main loop
detector = apriltag.Detector()
cam = cv2.VideoCapture(0)

screen_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
screen_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
midpoint = screen_width // 2

#arduino = serial.Serial(port='COM3', baudrate=115200, timeout=.1)
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.1)
left_desired_vel = 0
right_desired_vel = 0
servo_desired_angle = 90

LINE_LENGTH = 5
CENTER_COLOR = (0, 255, 0)
CORNER_COLOR = (255, 0, 255)
cone_position = 0

### Some utility functions to simplify drawing on the camera feed
# draw a crosshair
def plotPoint(image, center, color):
    center = (int(center[0]), int(center[1]))
    image = cv2.line(image,
                     (center[0] - LINE_LENGTH, center[1]),
                     (center[0] + LINE_LENGTH, center[1]),
                     color,
                     3)
    image = cv2.line(image,
                     (center[0], center[1] - LINE_LENGTH),
                     (center[0], center[1] + LINE_LENGTH),
                     color,
                     3)
    return image

def sendArduino(left_velocity, right_velocity, servo_angle):
    # if arduino.in_waiting > 0:
    # print(left_velocity, right_velocity, servo_angle)
    msg = f"{left_velocity},{right_velocity},{servo_angle}\n".encode() # encode message as bytes
    arduino.write(msg)

while True:
    # Read the frame from the video capture
    result, image = cam.read()
    grayimg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	# look for tags
    detections = detector.detect(grayimg)
    if not detections:
        print("Nothing")
    else:
	    # found some tags, report them and update the camera image
        #for detect in detections:
            #print("tag_id: %s, center: %s" % (detect.tag_id, detect.center))
            #image = plotPoint(image, detect.center, CENTER_COLOR)
            #image = plotText(image, detect.center, CENTER_COLOR, detect.tag_id)
            #for corner in detect.corners:
                #image = plotPoint(image, corner, CORNER_COLOR)
        cone_position = detections[0].center
	# refresh the camera image
    #cv2.imshow('Result', image)
	# let the system event loop do its thing
    print(cone_position)
    #go straight
    """
    if len(cone_position) == 0:
        left_desired_vel = -3
        right_desired_vel = 3
        print("No tag")
    elif(abs(float(cone_position[0]) - midpoint)< midpoint/6):
        left_desired_vel = -3
        right_desired_vel = -3
        print("tag in front")
    #go left
    elif(float(cone_position[0]) < midpoint):
        left_desired_vel = -3
        right_desired_vel = -1
        print("tag in left")
    #go right
    else:
        left_desired_vel = -1
        right_desired_vel = -3
        print("tag in right")
    """

    #main loop to constantly run through: updates arduino with motor commands when ready
    if arduino.in_waiting > 0:
        #left_desired_vel = input("Left vel: ")
        #right_desired_vel = input("Right vel: ")
        #servo_desired_angle = input("Servo angle: ")
        response = arduino.readline().decode().strip()
        print("Received from Arduino:", response)
        sendArduino(left_desired_vel,right_desired_vel,servo_desired_angle)

    key = cv2.waitKey(100)

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    

# Release the video capture and close all windows
cam.release()
cv2.destroyAllWindows()
