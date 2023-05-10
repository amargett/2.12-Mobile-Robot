import cv2
import numpy as np
import socket

# IP address and port for the receiver
receiver_ip = '10.29.100.148'  # Replace with the receiver's IP address
receiver_port = 12345  # Replace with the desired port number

# Open the video capture
cap = cv2.VideoCapture(0)  # Use the appropriate camera index if multiple cameras are connected

# Check if the camera opened successfully
if not cap.isOpened():
    print("Failed to open camera")
    exit()

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Check if the frame is valid
    if not ret:
        print("Failed to capture frame")
        break

    # Convert the frame to a byte array
    frame_data = frame.tobytes()

    # Send the frame data to the receiver
    sock.sendto(frame_data, (receiver_ip, receiver_port))

    # Display the frame locally
    cv2.imshow('Camera Stream', frame)

    # Exit if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
sock.close()
