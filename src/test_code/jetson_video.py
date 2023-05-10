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

# Constants for packet size and buffer size
packet_size = 65507  # Adjust packet size as needed
buffer_size = 65536  # Adjust buffer size as needed

# Buffer to store the received frame data
received_data = bytearray()

# Loop to receive and process frames
while True:
    # Receive data from the socket
    packet, sender_address = sock.recvfrom(buffer_size)

    # Append the received packet to the buffer
    received_data += packet

    # Check if the complete frame has been received
    if len(packet) < packet_size:
        # Convert the received data to a NumPy array
        frame = np.frombuffer(received_data, dtype=np.uint8)

        # Reshape the array to the original frame dimensions
        frame = frame.reshape((height, width, channels))  # Replace with the actual frame dimensions

        # Process the received frame
        # Example: Display the received frame
        cv2.imshow('Received Frame', frame)

        # Reset the buffer for the next frame
        received_data = bytearray()

    # Exit if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cv2.destroyAllWindows()
sock.close()
