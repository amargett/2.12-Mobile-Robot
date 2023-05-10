import socket

# IP address and port of the Linux computer
host = '10.29.100.148'
port = 12345

# Float values to send
float_values = [1.23, 4.56, 7.89]

# Create a socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the Linux computer
sock.connect((host, port))

# Send the float values
for value in float_values:
    sock.sendall(str(value).encode())

# Receive the float values from the Linux computer
received_values = []
for _ in range(3):
    data = sock.recv(1024).decode()
    received_values.append(float(data))

# Print the received float values
print('Received values:', received_values)

# Close the socket
sock.close()
