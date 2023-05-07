import serial
import time

# arduino = serial.Serial(port='dev/TTYUSB0', baudrate=115200, timeout=.1)
arduino = serial.Serial(port='/dev/tty.usbserial-0264FEA5', baudrate=115200, timeout=.1)

VEL = 3
pickup_angle = 90
target_x = 1
target_y = 1
alpha = 0.2

def readArduino():
    '''
    Read output from Arduino: x, y, heading.
    Returns [None, None, None] if response is not in the right format
    '''
    if arduino.in_waiting > 0:
        try:
            response = arduino.readline().decode().strip().split(',')
            if len(response) == 3 and all(len(i) > 0 for i in response):
                response = [float(i) for i in response]
                return response
        except:
            pass
    return [None, None, None]

def sendArduino(left_velocity, right_velocity, servo_angle):
    # if arduino.in_waiting > 0:
    # print(left_velocity, right_velocity, servo_angle)
    msg = f"{left_velocity},{right_velocity},{servo_angle}\n".encode() # encode message as bytes
    arduino.write(msg)
    
def go_straight(leftVel, rightVel, servoAngle):
    leftVel = -VEL
    rightVel = -VEL
    return leftVel, rightVel, servoAngle

def turn_left(leftVel, rightVel, servoAngle):
    leftVel = -VEL
    rightVel = VEL
    return leftVel, rightVel, servoAngle

def turn_right(leftVel, rightVel, servoAngle):
    leftVel = VEL
    rightVel = -VEL
    return leftVel, rightVel, servoAngle

def stop(leftVel, rightVel, servoAngle):
    leftVel = 0
    rightVel = 0
    return leftVel, rightVel, servoAngle

def pickup_aed(leftVel, rightVel, servoAngle):
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

def main():
    x0, y0, heading0 = [None, None, None]
    while [x0, y0, heading0] == [None, None, None]: # wait until readArduino receives usable data
        x0, y0, heading0 = readArduino()
    x, y, heading = [0, 0, 0]
    dx, dy, dheading = [0, 0, 0]
    leftVel, rightVel, servoAngle = [0, 0, 90]
    filtLeftVel, filtRightVel, filtServoAngle = [0, 0, 90]
    state = 0
    success = False
    prev_time = time.time()
    while True:
        x, y, heading = readArduino()
        # continue looping until readArduino receives usable data and at least 5 milliseconds have passed
        if [x, y, heading] != [None, None, None] and (time.time() - prev_time) > 5e-3: 
            dx = x0 - x
            dy = y0 - y
            dheading = heading - heading0
            if state == 0: # go straight
                leftVel, rightVel, servoAngle = go_straight(leftVel, rightVel, servoAngle)
                if dy >= target_y: # reached target y
                    state = 1
            elif state == 1: # turn left
                leftVel, rightVel, servoAngle = turn_left(leftVel, rightVel, servoAngle)
                if 180 <= dheading and dheading <= 270: # turned 90 degrees
                    state = 2
            elif state == 2: # go straight
                leftVel, rightVel, servoAngle = go_straight(leftVel, rightVel, servoAngle)
                if dx >= target_x: # reached target x
                    state = 3
            elif state == 3: # turn left
                leftVel, rightVel, servoAngle = turn_left(leftVel, rightVel, servoAngle)
                if 90 <= dheading and dheading <= 180: # turned 90 degrees
                    leftVel, rightVel, servoAngle = stop(leftVel, rightVel, servoAngle)
                    success = True

            prev_time = time.time()
            print(state, dx, dy, dheading)
            filtLeftVel = alpha*leftVel + (1 - alpha)*filtLeftVel
            filtRightVel = alpha*rightVel + (1 - alpha)*filtRightVel
            servoAngle = alpha*servoAngle + (1 - alpha)*filtServoAngle
            sendArduino(filtLeftVel, filtRightVel, filtServoAngle)
            if success is True:
                print('Success!')
                return
        else:
            pass
            
        
if __name__ == "__main__":
    main()