
import time

import cv2
import numpy as np

# Define the lower and upper bounds of the orange color in HSV color space
orange_lower = np.array([5, 100, 100])
orange_upper = np.array([15, 255, 255])

# Open the video capture
cap = cv2.VideoCapture(0)

screen_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
screen_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
midpoint = screen_width // 2

left_desired_vel = 0
right_desired_vel = 0
servo_desired_angle = 90


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
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize the position of the cone
    cone_position = None

    # Process the contours
    if len(contours) > 0:
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)

        # Calculate the center of the contour
        M = cv2.moments(largest_contour)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cone_position = (cx, cy)

            # Draw a circle at the center of the contour
            cv2.circle(frame, cone_position, 5, (0, 255, 0), -1)

    hsv_value = hsv[cy, cx]
    print(hsv_value)

    # Display the frame with the cone position
    cv2.imshow("Traffic Cone Detection", frame)

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    #go straight
    if not cone_position:
        continue
    """
    if(abs(cone_position[0] - midpoint)< midpoint/6):
        print("go straight")
        left_desired_vel = -3
        right_desired_vel = -3
    #go left
    elif(cone_position[0] < midpoint):
        print("go left")
        left_desired_vel = -3
        right_desired_vel = -1
    #go right
    else:
        print("go right")
        left_desired_vel = -1
        right_desired_vel = -3
    """

# Release the video capture and close all windows
cap.release()
cv2.destroyAllWindows()
