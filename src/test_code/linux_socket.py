import socket

# IP address and port to listen on
host = '192.168.239.1'
port = 12345

# Create a socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address
sock.bind((host, port))

# Listen for incoming connections
sock.listen(1)

# Accept a connection
conn, addr = sock.accept()
print('Connected by', addr)

# Receive the float values from the Jetson device
received_values = []
for _ in range(3):
    data = conn.recv(1024).decode()
    received_values.append(float(data))

# Print the received float values
print('Received values:', received_values)

# Float values to send back
float_values = [9.87, 6.54, 3.21]

# Send the float values back to the Jetson device
for value in float_values:
    conn.sendall(str(value).encode())

# Close the connection and socket
conn.close()
sock.close()
