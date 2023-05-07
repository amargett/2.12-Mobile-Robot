import serial
import time

VEL = 1

at_target = False
start = True
targetx = 1
targety = 0.5
targetheading = 90

pickup_angle = 120 # idk the right value for this @brayden

# arduino = serial.Serial(port='dev/TTYUSB0', baudrate=115200, timeout=.1)
arduino = serial.Serial(port='/dev/tty.usbserial-0264FEA5', baudrate=115200, timeout=.1)


def readArduino():
    # if arduino.in_waiting > 0:
    response = arduino.readline().decode().strip().split(',')
    # print(response)
    response = [float(i) for i in response]
    return response

def sendArduino(left_velocity, right_velocity, servo_angle):
    # if arduino.in_waiting > 0:
    # print(left_velocity, right_velocity, servo_angle)
    msg = f"{left_velocity},{right_velocity},{servo_angle}\n".encode() # encode message as bytes
    arduino.write(msg)

def setVel(state): 
#    """ take in state variable and send velocity vals to arduino"""
    leftVel = 0
    rightVel = 0
    servoAngle = 90
    if state == 0: # go forward
        leftVel = -VEL
        rightVel = -VEL
    elif state == 1: # turn left
        leftVel = -VEL
        rightVel = VEL
    elif state == 2: # turn right 
        leftVel = VEL
        rightVel = -VEL
    elif state == 3: # stop
        leftVel = 0
        rightVel = 0
    elif state == 4: # pickup aed
        servoAngle = pickup_angle
    return leftVel, rightVel, servoAngle

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
    leftVel, rightVel, servoAngle = setVel(state)
    sendArduino(leftVel, rightVel, servoAngle)
    response = readArduino()
    return response

def main():

    while arduino.in_waiting <= 0:
        pass
    x0, y0, heading0 = readArduino()
    # print(x0, y0, heading0)
    x = 0
    y = 0
    heading = 0
    dy = 0
    dx = 0
    dheading = 0
    while dy<targety:
        # if obstacle(): 
        #     go_around(1)
        if arduino.in_waiting > 0:
            x, y, heading = update(0)
        dy = y0 - y
        print(0, dx, dy, heading)
    while dheading < 90: 
        if arduino.in_waiting > 0:
            x, y, heading = update(1)
        dheading = heading - heading0
        print(1, dx, dy, heading)
    while dx < targetx: 
        if arduino.in_waiting > 0:
            x, y, heading = update(0)
        dx = x0 - x
        print(2, dx, dy, heading)
    while dheading < 180: 
        if arduino.in_waiting > 0:
            x, y, heading = update(1)
        dheading = heading - heading0
        print(3, dx, dy, heading)
    while not close(): 
        if arduino.in_waiting > 0:
            x, y, heading = update(0)

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
