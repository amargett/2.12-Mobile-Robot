import serial
import time

# arduino = serial.Serial(port='dev/TTYUSB0', baudrate=115200, timeout=.1)
arduino = serial.Serial(port='/dev/tty.usbserial-0264FEA5', baudrate=115200, timeout=.1)

STRAIGHT_VEL = 3
TURN_VEL = STRAIGHT_VEL/2
pickup_angle = 90
target_x = 1
target_y = 1.65
alpha = 0.15
epsilon_heading = 2
epsilon_dist = 0.1

def april_tag(): 
    # detects whether there is an april tag in view
    return False

def obstacle(): 
    # CV, determine whether or not there is an obstacle there
    return False

def close(): 
    # ToF sensor, will decide whether we are close enough to AED
    return True

def turnaround(car):  
    mini_state = 0
    while True:
        if mini_state == 0: 
            car.back()
            if abs(car.x - car.x0) >= target_x:
                mini_state =1
        if mini_state == 1: 
            car.left()
            dheading = abs(car.heading - car.heading0)
            if 29 < dheading < 31: 
                car.stop()
                return car
        car.prev_time = time.time()
        car.printCurr()
        car.filter()
        car.send()


def main():
    car = Car()
    while car.getArduino() == [None, None, None]: # wait until readArduino receives usable data
        car.x0, car.y0, _ = car.readArduino()
    while True:
        car.readArduino()
        # continue looping until readArduino receives usable data and at least 5 milliseconds have passed
        if [car.x_raw, car.y_raw, car.heading_raw] != [None, None, None] and (time.time() - car.prev_time) > 5e-3: 
            car.setXYH()
        if car.state == 0: ## go to AED
            # go to a point (x, y)
            car = go(target_x, target_y, car)
            if car.success:
                car.pickupAED()
                car.send()
                print('Success! AED picked up')
                car.state = 1
                return
            pass
        elif car.state == 1: ## turning around
            car.back()
                
class Car(object): 
    def __init__(self): 
        self.leftVel = 0
        self.rightVel = 0
        self.servoAngle = 90

        self.x0 = None
        self.y0 = None
        self.heading0 = None

        self.x = 0
        self.y = 0
        self.heading = 0

        self.x_raw = 0
        self.y_raw = 0
        self.heading_raw = 0

        self.filtLeftVel = 0
        self.filtRightVel = 0
        self.filtServoAngle = 90

        self.state = 0
        self.prev_time = time.time()
        self.success = False

        self.pickup_counter = 0
        self.dropoff_counter = 0

    def readArduino(self):
        '''
        Read output from Arduino: x, y, heading.
        Returns [None, None, None] if response is not in the right format
        '''
        if arduino.in_waiting > 0:
            try:
                response = arduino.readline().decode().strip().split(',')
                if len(response) == 3 and all(len(i) > 0 for i in response):
                    response = [float(i) for i in response]
                    self.x_raw, self.y_raw, self.heading_raw = response
            except:
                pass
        self.x_raw, self.y_raw, self.heading_raw = [None, None, None]
        return self.x_raw, self.y_raw, self.heading_raw
    
    def straight(self): 
        self.leftVel, self.rightVel = -STRAIGHT_VEL, -STRAIGHT_VEL

    def left(self): 
        self.leftVel, self.rightVel = -TURN_VEL, TURN_VEL

    def left(self): 
        self.leftVel, self.rightVel = TURN_VEL, -TURN_VEL

    def stop(self):
        self.leftVel, self.rightVel = 0, 0
    
    def back(self): 
        self.leftVel, self.rightVel = STRAIGHT_VEL, STRAIGHT_VEL

    def pickupAED(self):
        self.servoAngle = 40

    def setVelAngle(self): 
        self.leftVel, self.rightVel, self.servoAngle = go()

    def setXYH(self): 
        self.x = self.x0 - self.x_raw
        self.y = self.y_raw - self.y0
        self.heading = self.heading_raw

    def printCurr(self): 
        print(self.state, self.x, self.y, self.heading)

    def filter(self): 
        self.filtLeftVel = alpha*self.leftVel + (1 - alpha)*self.filtLeftVel
        self.filtRightVel = alpha*self.rightVel + (1 - alpha)*self.filtRightVel
        self.filtServoAngle = alpha*self.servoAngle + (1 - alpha)*self.filtServoAngle

    def send(self): 
        msg = f"{self.filtLeftVel},{self.filtRightVel},{self.filtServoAngle}\n".encode() # encode message as bytes
        arduino.write(msg)
    
    def go(self, target_x, target_y, mini_state): 
        '''
        Args:
            target_x(float)
            target_y(float)
            car(Car object)

        Returns:
            mini_state
        '''
        # mini_state == 0: rotate
        # mini_state == 1: go forward
        # mini_state = 2: success!
        # target_heading 
        while True:
            if mini_state == 0: # rotate
                self.left()
                
            elif mini_state == 1: # go straight
                if mini_state == 0: # go straight
                    self.straight()
                    if abs(self.y-self.y0) >= target_y: # reached target y
                        mini_state = 1
                elif mini_state == 1: # turn left
                    self.left()
                    dheading = abs(self.heading - self.heading0)
                    if 180 <= dheading and dheading <= 270: # turned 90 degrees
                        mini_state = 2
                elif mini_state == 2: # go straight
                    self.straight()
                    if abs(self.x - self.x0) <= target_x: # reached target x
                        mini_state = 3
                elif mini_state == 3: # turn left
                    self.left()
                    dheading = abs(self.heading - self.heading0)
                    if 90 <= dheading and dheading <= 180: # turned 90 degrees
                        self.stop()
                        return self
                self.prev_time = time.time()
                self.printCurr()
                self.filter()
                self.send()

if __name__ == "__main__":
    main()