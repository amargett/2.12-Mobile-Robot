import socket
import cv2
import numpy as np

# IP address and port for receiving frames
receiver_ip = '0.0.0.0'  # Listen on all available network interfaces
receiver_port = 12345  # The same port number used on the Jetson device

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the receiver IP address and port
sock.bind((receiver_ip, receiver_port))

# Loop to receive and process frames
while True:
    # Receive data from the socket
    frame_data, sender_address = sock.recvfrom(65536)  # Adjust the buffer size as needed

    # Convert the byte array to a NumPy array
    frame = np.frombuffer(frame_data, dtype=np.uint8)

    # Reshape the array to the original frame dimensions
    frame = frame.reshape((1920, 1080, 3))  # Replace with the actual frame dimensions

    # Process the received frame
    # Example: Display the received frame
    cv2.imshow('Received Frame', frame)

    # Exit if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cv2.destroyAllWindows()
sock.close()
