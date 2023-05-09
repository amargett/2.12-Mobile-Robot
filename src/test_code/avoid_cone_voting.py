import serial
import time

import cv2
import numpy as np

# Define the lower and upper bounds of the orange color in HSV color space
orange_lower = np.array([1, 100, 150])
orange_upper = np.array([15, 200, 255])
#0 straight -1 left 1 right
last_pos = 0

# Open the video capture
cap = cv2.VideoCapture(0)

is_cone = 0

screen_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
screen_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
midpoint = screen_width // 2
vote_array = []

#arduino = serial.Serial(port='COM3', baudrate=115200, timeout=.1)
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.1)
left_desired_vel = 0
right_desired_vel = 0
servo_desired_angle = 90

des_vel = 2

cone_position = (0,0)


def sendArduino(left_velocity, right_velocity, servo_angle):
    # if arduino.in_waiting > 0:
    # print(left_velocity, right_velocity, servo_angle)
    msg = f"{left_velocity},{right_velocity},{servo_angle}\n".encode() # encode message as bytes
    arduino.write(msg)

while True:
    # Read the frame from the video capture
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame from camera")
        break

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
    cone_position = None
    cone_detected = 0
    state = 0
    # Process the contours
    if len(contours) > 0:
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)

        # Calculate the area of the largest contour
        largest_contour_area = cv2.contourArea(largest_contour)
        print("largest_contour", largest_contour_area)

        # Check if the largest contour has a large enough magnitude (area)
        if largest_contour_area > 2500:
            # Calculate the center of the contour
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)

                # Print the position of the cone
                #print("Cone position: ({}, {})".format(cx, cy))
                
                cone_detected = 1
                
                if cx < midpoint:
                    print("cone on left")
                    state = 1
                    #left_desired_vel = -1
                    #right_desired_vel = -3
                    
                else:
                    print("cone on right")
                    state = 2
                    #left_desired_vel = -3
                    #right_desired_vel = -1
                
                cone_position = (cx, cy)

                
                
        else:
            print("no cone")
            #left_desired_vel = -3
            #right_desired_vel = -3
        
    vote_array.append(cone_detected)
        

        

    # If the vote array has more than 9 elements, remove the oldest one
    if len(vote_array) > 15:
        vote_array.pop(0)


    # Determine the state of the current frame based on the vote array
    """
    if sum(vote_array) > len(vote_array) / 2 and state == 1:
        print ("cone detected on left_determined")
        left_desired_vel = -1
        right_desired_vel = -3
        
    elif sum(vote_array) > len(vote_array) / 2 and state == 2:
        print ("cone detected on right_determined")
        left_desired_vel = -3
        right_desired_vel = -1
        
    elif sum(vote_array) > len(vote_array) / 2 and state == 0:
        print ("maybe false negative") # maybe change to sticking with previous state, but take as no cone rn
        left_desired_vel = -3
        right_desired_vel = -3
        
    else:
        print ("no cone determined")
        left_desired_vel = -3
        right_desired_vel = -3
    """
    if sum(vote_array) < len(vote_array) / 2:
        print("no cone detected, or not enough votes")
        left_desired_vel = -des_vel
        right_desired_vel = des_vel
    else:
        if(cone_position[0] < midpoint):
            fraction_diff = cone_position[0]/midpoint
        else:
            fraction_diff = cone_position[0]/midpoint - 2
        left_desired_vel = -des_vel - 4* fraction_diff
        right_desired_vel = -des_vel + 4* fraction_diff
            
    # Display the frame with the largest contour position
    cv2.imshow("Orange Cone Detection", mask)

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    
    #main loop to constantly run through: updates arduino with motor commands when ready
    if arduino.in_waiting > 0:
        #left_desired_vel = input("Left vel: ")
        #right_desired_vel = input("Right vel: ")
        #servo_desired_angle = input("Servo angle: ")
        response = arduino.readline().decode().strip()
        print("Received from Arduino:", response)
        sendArduino(left_desired_vel,right_desired_vel,servo_desired_angle)
    
# Release the video capture and close all windows
cap.release()
cv2.destroyAllWindows()
