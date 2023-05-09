import cv2
import numpy as np

def find_orange_cone():
    # Open the webcam
    cap = cv2.VideoCapture(0)

    # Get the screen resolution
    screen_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    screen_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    midpoint = screen_width // 2

    while True:
        # Read the frame from the webcam
        ret, frame = cap.read()
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
        _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.imshow("Orange Cone Detection", mask)
        # Initialize the position of the largest contour
        largest_contour_position = None

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

                    # Print the position of the cone
                    #print("Cone position: ({}, {})".format(cx, cy))
                    """
                    if cx < midpoint:
                        print("cone on left")
                    else:
                        print("cone on right")
                    """
            else:
                print("no cone")
        # Display the frame with the largest contour position
        #cv2.imshow("Orange Cone Detection", frame)

        # Exit the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all windows
    cap.release()
    cv2.destroyAllWindows()

find_orange_cone()
