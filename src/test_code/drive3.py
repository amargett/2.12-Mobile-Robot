import serial
import time

# arduino = serial.Serial(port='dev/TTYUSB0', baudrate=115200, timeout=.1)
arduino = serial.Serial(port='/dev/tty.usbserial-0264FEA5', baudrate=115200, timeout=.1)

STRAIGHT_VEL = 3
TURN_VEL = STRAIGHT_VEL/2
pickup_angle = 90
target_x = 1
target_y = 1
alpha = 0.15
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

def april_tag(): 
    # detects whether there is an april tag in view
    return False

def obstacle(): 
    # CV, determine whether or not there is an obstacle there
    return False

def close(): 
    # ToF sensor, will decide whether we are close enough to AED
    return True

def go(target_x, target_y, car, ob = False): 
    '''
    Args:
        target_x(float)
        target_y(float)
        car(Car object)
        ob(whether or not this is called from an obstacle, bool)

    Returns:
        car (Car object)
    '''
    # mini_state == 0: drive straight until x/y value
    # mini_state == 1: turn towards x value
    # mini_state == 2: drive straight until x/y value
    # mini_state == 3: turn towards goal heading
    mini_state = 0
    while True: 
        if abs(target_x - car.x) < epsilon_dist and abs(target_y - car.y) < epsilon_dist: 
            if not ob and obstacle(): 
                go(car.x + 0.5, car.y+0.5, car, True)
            else: 
                if mini_state == 0: # go straight
                    car.straight()
                    if abs(car.y-car.y0) >= target_y: # reached target y
                        mini_state = 1
                elif mini_state == 1: # turn left
                    car.left()
                    dheading = abs(car.heading - car.heading0)
                    if 180 <= dheading and dheading <= 270: # turned 90 degrees
                        mini_state = 2
                elif mini_state == 2: # go straight
                    car.straight()
                    if abs(car.x - car.x0) <= target_x: # reached target x
                        mini_state = 3
                elif mini_state == 3: # turn left
                    car.left()
                    dheading = abs(car.heading - car.heading0)
                    if 90 <= dheading and dheading <= 180: # turned 90 degrees
                        car.stop()
                        return car
def main():
    car = Car()
    while car.getArduino0 == [None, None, None]: # wait until readArduino receives usable data
        car.setArduino0()
    while True:
        car.setArduinoRaw()
        # continue looping until readArduino receives usable data and at least 5 milliseconds have passed
        if car.getArduinoRaw() != [None, None, None] and (time.time() - car.prev_time) > 5e-3: 
            car.setXYH()
        if car.state == 0: ## attempting to go to aed
            # go to a point (x, y)
            car = go(target_x, target_y, car)
            car.prev_time = time.time()
            car.printCurr()
            car.filter()
            car.send()
            if car.success:
                car.pickupAED()
                car.send()
                print('Success! AED picked up')
                car.state = 1
                return
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
    
    def straight(self): 
        self.leftVel, self.rightVel = -STRAIGHT_VEL, -STRAIGHT_VEL

    def left(self): 
        self.leftVel, self.rightVel = -TURN_VEL, TURN_VEL

    def left(self): 
        self.leftV

    def stop(self):
        self.leftVel, self.rightVel = 0, 0

    def pickupAED(self): 
        self.servoAngle = 40

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
    def send(self): 
        msg = f"{self.filtLeftVel},{self.filtRightVel},{self.filtServoAngle}\n".encode() # encode message as bytes
        arduino.write(msg)

if __name__ == "__main__":
    main()