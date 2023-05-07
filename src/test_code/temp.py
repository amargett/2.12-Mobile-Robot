def main():
    # state 0: go forward
    # state 1: turn left
    # state 2: go forward
    # state 3: turn left
    
    x0, y0, heading0 = readArduino()

    state = 0

    while True:
        dy = y - y0
        dx = y - x0
        dheading = heading - heading0
        if state == 0:
            setVel(0)
            if dy >= targety:
                state = 1
        elif state == 1:
            setVel(1)
            if dheading >= 90:
                state = 2
        elif state == 2:
            setVel(2)
            if dx >= targetx:
                state = 3 
        elif state == 3:
            setVel(3)
            if dheading >= 180:
                print('Success!')
                return
        sendArduino()
        readArduino()