import serial
import time

VEL = 0.3

at_target = False
start = True
targetx = 1 
targety = 2
targetheading = 90

leftVel = 0
rightVel = 0
servoAngle = 90

x = 0
y = 0
heading = 0

x0 = 0
y0 = 0
heading0 = 0

pickup_angle = 120 # idk the right value for this @brayden

arduino = serial.Serial(port='dev/TTYUSB0', baudrate=115200, timeout=.1)

def readArduino(): 
    if arduino.in_waiting > 0:
        response = arduino.readline().decode().strip().split(',')
        x, y, heading = response
        return response

def sendArduino():
    if arduino.in_waiting > 0:
        msg = f"{leftVel},{rightVel},{servoAngle}\n".encode() # encode message as bytes
        arduino.write(msg)

def setVel(state): 
#    """ take in state variable and send velocity vals to arduino"""
    if state == 0: # go forward
        leftVel = VEL
        rightVel = VEL
    if state == 1: # turn left
        leftVel = -VEL
        rightVel = VEL
    if state == 2: # turn right 
        leftVel = VEL
        rightVel = -VEL
    if state == 3: # stop
        leftVel = 0
        rightVel = 0
    if state == 4: # pickup aed
        servoAngle = pickup_angle

def april_tag(): 
    # implement, detects whether there is an april tag in view
    return True

def obstacle(): 
    # CV, determine whether or not there is an obstacle there
    return True

def close(): 
    # ToF sensor, will decide whether we are close enough to AED
    return True

# def go_around(direction): 
#     while obstacle(): 
#         setVel(direction)
#     setVel(0)
#     time.sleep(1)
#     readArduino()
#     dtheta = heading - (heading0 + 90)
#     if dtheta > 0: 
#         direction = 1
#     else: 
#         direction = 2
#     while heading != heading0 +90: 
#         setVel(direction)

def update(state): 
    setVel(state)
    sendArduino()
    readArduino()

def main():
    x0, y0, heading0 = readArduino()
    dy = 0 
    dx = 0
    dheading = 0
    while dy<targety:
        # if obstacle(): 
        #     go_around(1)
        update(0)
        dy = y - y0
    while dheading < 90: 
        update(1)
        dheading = heading - heading0
    while dx < targetx: 
        update(0)
        dx = x - x0 
    while dheading < 180: 
        update(1)
        dheading = heading - heading0
    while not close(): 
        update(0)

    # if obstacle(): 
    #     go_around()

    # GO STRAIGHT UNTIL Y
    # TURN LEFT
    # GO STRAIGHT UNTIL X
    # TURN LEFT
    # TURN UNTIL AED IN THE MIDDLE
    # GO STRAIGHT UNTIL TOF REALLY CLOSE

if __name__ == "__main__":
    main()
