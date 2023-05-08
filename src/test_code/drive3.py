import serial
import time
import numpy as np

arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.1)
# arduino = serial.Serial(port='/dev/tty.usbserial-0264FEA5', baudrate=115200, timeout=.1)

STRAIGHT_VEL = 5
TURN_VEL = STRAIGHT_VEL/2
PICKUP_ANGLE = 40
ALPHA = 0.15
EPSILON_HEADING = 1
EPISLON_DIST = 0.1
K_HEADING = 0.017

def april_tag(): 
    # detects whether there is an april tag in view
    return False

def obstacle(): 
    # CV, determine whether or not there is an obstacle there
    return False

def close(): 
    # ToF sensor, will decide whether we are close enough to AED
    return True

# def turnaround(car):  
#     mini_state = 0
#     while True:
#         if mini_state == 0: 
#             car.back()
#             if abs(car.x - car.x0) >= target_x:
#                 mini_state =1
#         if mini_state == 1: 
#             car.left()
#             dheading = abs(car.heading - car.heading0)
#             if 29 < dheading < 31: 
#                 car.stop()
#                 return car
#         car.prev_time = time.time()
#         car.printCurr()
#         car.filter()
#         car.send()
def main():
    car = Car()
    while [car.x0, car.y0, car.heading0] == [None, None, None]: # wait until readArduino receives usable data
        car.x0, car.y0, car.heading0 = car.readArduino()
    while True:
        car.readArduino()
        # continue looping until readArduino receives usable data and at least 5 milliseconds have passed
        # if obstacle():
        #     car.go()
        if [car.x_raw, car.y_raw, car.heading_raw] != [None, None, None] and (time.time() - car.prev_time) > 5e-3: 
            car.setXYH()
            if car.state == 0: ## go to AED waypoint
                car.target_x = 1
                car.target_y = 1.65
                car.mini_state = car.go(car.mini_state)
                if car.mini_state == 2:
                    car.mini_state = 0
                    car.state = 1
            if car.state == 1: # go to AED
                car.target_x = 0
                car.target_y = 1.65
                car.mini_state = car.go(car.mini_state)
                if car.mini_state == 2:
                    car.success = True
    
            car.prev_time = time.time()
            car.printCurr()
            if car.success:
                print('Success! AED picked up')
                car.stop()
                car.pickupAED()
            car.filter()
            car.sendArduino()
                
class Car(object): 
    def __init__(self): 
        self.target_x = 1
        self.target_y = 1.65
        self.target_heading = 0
        
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

        self.mini_state = 0

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

    def left(self, error): 
        self.leftVel, self.rightVel = -K_HEADING*error, K_HEADING*error

    def right(self, error): 
        self.leftVel, self.rightVel = K_HEADING*error, -K_HEADING*error

    def stop(self):
        self.leftVel, self.rightVel = 0, 0
    
    def back(self): 
        self.leftVel, self.rightVel = STRAIGHT_VEL, STRAIGHT_VEL

    def pickupAED(self):
        self.servoAngle = PICKUP_ANGLE

    def setVelAngle(self): 
        self.leftVel, self.rightVel, self.servoAngle = go()

    def setXYH(self): 
        self.x = self.x0 - self.x_raw
        self.y = self.y_raw - self.y0
        self.heading = self.heading_raw

    def printCurr(self):
        print('State: %f, x: %f, y: %f, heading: %f, servoAngle: %f' % (self.state, self.x, self.y, self.heading, self.servoAngle))

    def filter(self): 
        self.filtLeftVel = ALPHA*self.leftVel + (1 - ALPHA)*self.filtLeftVel
        self.filtRightVel = ALPHA*self.rightVel + (1 - ALPHA)*self.filtRightVel
        self.filtServoAngle = ALPHA*self.servoAngle + (1 - ALPHA)*self.filtServoAngle

    def sendArduino(self): 
        msg = f"{self.filtLeftVel},{self.filtRightVel},{self.filtServoAngle}\n".encode() # encode message as bytes
        arduino.write(msg)
    
    def go(self, mini_state): 
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
        self.target_heading = np.arctan2(self.target_y/self.target_x)

        while True:
            if mini_state == 0: # rotate
                dheading = abs(self.heading-self.target_heading)
                if self.target_x>0: 
                    self.left(dheading)
                else: 
                    self.right(dheading)
                if abs(self.heading - self.target_heading) < EPSILON_HEADING:
                    mini_state = 1
            if mini_state == 1: # go straight
                self.straight()
                if abs(self.x - self.target_x) < EPISLON_DIST and abs(self.y - self.target_y) < EPISLON_DIST:
                    mini_state = 2
            return mini_state
                

if __name__ == "__main__":
    main()