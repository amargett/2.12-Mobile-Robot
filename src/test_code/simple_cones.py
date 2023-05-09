import cv2
import numpy as np
import time


def find_orange_cone():
    # Open the webcam
    cap = cv2.VideoCapture(0)

    # Get the screen resolution
    screen_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    screen_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    midpoint = screen_width // 2

    # Initialize variables for frame voting
    #frame_count = 0
    #cone_detected_count = 0
    vote_array = []
    

    while True:
        # Read the frame from the webcam
        ret, frame = cap.read()
        #frame_count += 1
        #print(frame_count)
        if not ret:
            print("Failed to capture frame from camera")
            break

        # Convert the frame to the HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds of the orange color in HSV color space
        orange_lower = np.array([1, 100, 150])
        orange_upper = np.array([15, 255, 255])

        # Create a mask based on the orange color range
        mask = cv2.inRange(hsv, orange_lower, orange_upper)

        # Perform morphological operations to remove noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize the position of the largest contour
        largest_contour_position = None

        cone_detected = 0
        state = 0
        # Process the contours
        if len(contours) > 0:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            # Calculate the area of the largest contour
            largest_contour_area = cv2.contourArea(largest_contour)

            # Check if the largest contour has a large enough magnitude (area)
            if largest_contour_area > 5000:
                # Calculate the center of the contour
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)

                    # Increment the cone detected count
                    cone_detected = 1
                    #print("cone_detected = ", cone_detected_count)

                    # Print the position of the cone
                    if cx < midpoint:
                        print("cone on left")
                        state = 1
                        
                    else:
                        print("cone on right")
                        state = 2
            else:
                print("no cone")
            #     # Add a 0 to the vote array if no cone is detected in this frame
            #     vote_array.append(0)
        # else:
        #     print("no cone")
        #     # Add a 0 to the vote array if no cone is detected in this frame
        #     vote_array.append(0)
            
        
        vote_array.append(cone_detected)
        

        

        # If the vote array has more than 9 elements, remove the oldest one
        if len(vote_array) > 15:
            vote_array.pop(0)
            

        # Determine the state of the current frame based on the vote array
        if sum(vote_array) > len(vote_array) / 2 and state == 1:
            print ("cone detected on left_determined")
        elif sum(vote_array) > len(vote_array) / 2 and state == 2:
            print ("cone detected on right_determined")
        elif sum(vote_array) > len(vote_array) / 2 and state == 0:
            print ("maybe false negative")
            
        else:
            print ("no cone determined")
            
        #time.sleep(1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all windows
    cap.release()
    cv2.destroyAllWindows()

find_orange_cone()
