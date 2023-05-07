import serial
import time

# arduino = serial.Serial(port='dev/TTYUSB0', baudrate=115200, timeout=.1)
arduino = serial.Serial(port='/dev/tty.usbserial-0264FEA5', baudrate=115200, timeout=.1)

VEL = 3
pickup_angle = 90
target_x = 1
target_y = 1
alpha = 0.2
epsilon_dist = 0.1
epsilon_heading = 1

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
    # detects whether there is an april tag in view
    return apriltag_detection()

def obstacle(): 
    # CV, determine whether or not there is an obstacle there
    return obstacle_detection()

def close(): 
    # ToF sensor, will decide whether we are close enough to AED
    return True

def go(x_i, y_i, heading_i, x_f, y_f, heading_f, mini_state, success): 
    '''
    Args:
        x_i (float)
        y_i (float)
        heading_i (float)
        x_f (float)
        y_f (float)
        heading_f (float)

    Returns:
        leftVel (float)
        rightVel (float)
        servoAngle (float)
        mini_state (int)
        success (bool)
    '''
    # mini_state == 0: drive straight until x/y value
    # mini_state == 1: turn towards x value
    # mini_state == 2: drive straight until x/y value
    # mini_state == 3: turn towards goal heading

    if abs(x_f - x_i) < epsilon_dist and abs(y_f - y_i) < epsilon_dist and heading_
    
    if mini_state == 0: # go straight
        leftVel, rightVel, servoAngle = go_straight(leftVel, rightVel, servoAngle)
        if dy >= target_y: # reached target y
            mini_state = 1
    elif mini_state == 1: # turn left
        leftVel, rightVel, servoAngle = turn_left(leftVel, rightVel, servoAngle)
        if 180 <= dheading and dheading <= 270: # turned 90 degrees
            mini_state = 2
    elif mini_state == 2: # go straight
        leftVel, rightVel, servoAngle = go_straight(leftVel, rightVel, servoAngle)
        if dx <= target_x: # reached target x
            mini_state = 3
    elif mini_state == 3: # turn left
        leftVel, rightVel, servoAngle = turn_left(leftVel, rightVel, servoAngle)
        if 90 <= dheading and dheading <= 180: # turned 90 degrees
            leftVel, rightVel, servoAngle = stop(leftVel, rightVel, servoAngle)
            success = True

def main():
    car = Car()
    while car.getArduino0 == [None, None, None]: # wait until readArduino receives usable data
        car.setArduino0()
    while True:
        car.setArduinoRaw()
        # continue looping until readArduino receives usable data and at least 5 milliseconds have passed
        if car.getArduinoRaw() != [None, None, None] and (time.time() - car.prev_time) > 5e-3: 
            car.setXYH()
        if car.state == 0:
            # go to a point (x, y)
            if obstacle(): 
                
                return
            car.setVelAngle() ### should pass in same arguments as go()
            car.prev_time = time.time()
            car.printCurr()
            car.filter()
            sendArduino(car.filtLeftVel, car.filtRightVel, car.filtServoAngle)
            if car.success:
                print('Success!')
                return
        else:
            pass
            
class Car(object): 
    def __init__(self): 
        self.leftVel = 0
        self.rightVel = 0
        self.servoAngle = 90

        self.x = 0
        self.y = 0
        self.heading = 0

        self.x0 = None
        self.y0 = None
        self.heading0 = None

        self.filtLeftVel = 0
        self.filtRightVel = 0
        self.filtServoAngle = 90

        self.x_raw = 0
        self.y_raw = 0
        self.heading_raw = 0

        self.state = 0
        self.prev_time = time.time()
        self.success = False

    def setArduino0(self): 
        self.x0, self.y0, self.heading0 = readArduino()
    def getArduino0(self): 
        return [self.x0, self.y0, self.heading0]
    def setArduinoRaw(self): 
        self.x_raw, self.y_raw, self.heading_raw = readArduino()
    def getArduinoRaw(self):
        return [self.x_raw, self.y_raw, self.heading_raw]
    def getArduino(self): 
        self.x, self.y, self.heading = readArduino()
    def getArduino(self):
        return [self.x, self.y, self.heading]

    def setVelAngle(self): 
        self.leftVel, self.rightVel, self.servoAngle = go()
    
    def setXYH(self): 
        self.x = self.x0 - self.x_raw
        self.y = self.y0 - self.y_raw
        self.heading = self.heading_raw - self.heading0

    def printCurr(self): 
        print(self.state, self.x, self.y, self.heading)

    def filter(self): 
        self.filtLeftVel = alpha*self.leftVel + (1 - alpha)*self.filtLeftVel
        self.filtRightVel = alpha*self.rightVel + (1 - alpha)*self.filtRightVel
        self.servoAngle = alpha*self.servoAngle + (1 - alpha)*self.filtServoAngle

if __name__ == "__main__":
    main()