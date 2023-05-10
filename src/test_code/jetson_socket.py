import socket
import json

# IP address and port of the Linux computer
host = '10.29.100.148'
port = 12345

# Create a socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def send_data(data):
    # Convert data to JSON string
    json_data = json.dumps(data)

    # Send the JSON string
    sock.sendall(json_data.encode())

    # Receive the JSON string
    received_data = sock.recv(1024).decode()

    # Parse the JSON string to get the received data
    received_data = json.loads(received_data)

    # Return the received data
    return received_data

# Connect to the Linux computer
sock.connect((host, port))
print('Connected to', host)

# Continuously send and receive data
while True:
    # Data to send
    data_to_send = [1.23, 4.56, 7.89]

    # Send and receive data
    received_data = send_data(data_to_send)

    # Print the received data
    print('Received data:', received_data)

# Close the socket
sock.close()
