from datetime import datetime
from datetime import timedelta

at_target = False
start = True
targetx = 1 
targety = 2


def send(state): 
    # implement, function to send a desired state to the arduino
    return None

def april_tag(): 
    # implement, detects whether there is an april tag in view
    return True

def get_imu(): 
    # implement, function to get imu data from the arduino
    x = 0
    y = 0
    heading = 0 ## change to be real values from arduino
    return [x, y, heading]

def obstacle(): 
    return True

def go_around(): 
    while obstacle(): 
        send(1)
    pos = get_imu()

## main loop 
if start: 
    start_imu = get_imu()
    dy = 0; 
    while dy<targety: 
        if obstacle(): 
            go_around()
        send(0)
        obstacle()
        dy = get_imu()[1] - start_imu[1]
    if obstacle(): 


            