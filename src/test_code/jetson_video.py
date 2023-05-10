import cv2
import numpy as np
import socket

# IP address and port for the receiver
receiver_ip = '10.29.100.148'  # Replace with the receiver's IP address
receiver_port = 12345  # Replace with the desired port number

# Maximum packet size to send over UDP
max_packet_size = 65507  # Adjust the value based on your network's MTU

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

    # Split the frame into smaller packets
    frame_bytes = frame.tobytes()
    num_packets = len(frame_bytes) // max_packet_size + 1
    for i in range(num_packets):
        start = i * max_packet_size
        end = (i + 1) * max_packet_size
        packet = frame_bytes[start:end]

        # Send the packet to the receiver
        sock.sendto(packet, (receiver_ip, receiver_port))

    # Display the frame locally
    cv2.imshow('Camera Stream', frame)

    # Exit if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
sock.close()
