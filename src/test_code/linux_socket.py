import socket
import json

# IP address and port to listen on
host = '10.29.100.148'
port = 12345

# Create a socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def receive_data():
    # Receive the JSON string
    data = conn.recv(1024).decode()

    # Parse the JSON string to get the received data
    received_data = json.loads(data)

    # Data to send back
    data_to_send = [9.87, 6.54, 3.21]

    # Convert data to JSON string
    json_data = json.dumps(data_to_send)

    # Send the JSON string back
    conn.sendall(json_data.encode())

    # Return the received data
    return received_data

# Bind the socket to the address
sock.bind((host, port))

# Listen for incoming connections
sock.listen(1)
print('Listening for connections...')

# Accept a connection
conn, addr = sock.accept()
print('Connected by', addr)

# Continuously receive and send data
while True:
    # Receive and send data
    received_data = receive_data()

    # Print the received data
    print('Received data:', received_data)

# Close the connection and socket
conn.close()
sock.close()
