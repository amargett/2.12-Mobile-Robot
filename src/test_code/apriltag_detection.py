import cv2
import numpy as np
import math
from pupil_apriltags import Detector

#LINE_LENGTH = 5
#CENTER_COLOR = (0, 255, 0)
#CORNER_COLOR = (255, 0, 255)

### Some utility functions to simplify drawing on the camera feed
# draw a crosshair
def plotPoint(image, center, color):
    center = (int(center[0]), int(center[1]))
    image = cv2.line(image,
                     (center[0] - LINE_LENGTH, center[1]),
                     (center[0] + LINE_LENGTH, center[1]),
                     color,
                     3)
    image = cv2.line(image,
                     (center[0], center[1] - LINE_LENGTH),
                     (center[0], center[1] + LINE_LENGTH),
                     color,
                     3)
    return image

# plot a little text
def plotText(image, center, color, text):
    center = (int(center[0]) + 4, int(center[1]) - 4)
    return cv2.putText(image, str(text), center, cv2.FONT_HERSHEY_SIMPLEX,
                       0.5, color, 3)


def detect_apriltag(frame):
    cap=cv2.VideoCapture(0)  #camera used
    detector = Detector(families='tag36h11', 
                    nthreads=1,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0,
                    ) #physical size of the apriltag

#intrisic = [777.60876405,777.24415999,385.3505718,320.0302134] # camera parameters, [fx, fy cx, cy]
    intrisic = [640,640,960,540]

    tagsize = 0.100  #physical size of printed tag, unit = meter

    threshold = 5  # tolerable yaw

    looping = True


while (looping):
    ret, frame = cap.read()
    frame = cv2.resize(frame, (640,480))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray, estimate_tag_pose=True, camera_params=intrisic, tag_size=tagsize)
    
    if not tags:
        print("Nothing")
    else:
        for tag in tags:
            #print(tag.pose_R)
            #print(tag.pose_t)
            #print(-np.linalg.inv(tag.pose_R)@ tag.pose_t)
	    #print(tag)
            #print("thetax =", math.atan(tag.pose_R[1,0]/tag.pose_R[0,0])/math.pi*180)
            #print("thetay =", math.atan(-tag.pose_R[2,0]/math.sqrt(tag.pose_R[2,1]*tag.pose_R[2,1]+tag.pose_R[2,2]*tag.pose_R[2,2]))/math.pi*180)
            #print("thetaz =", math.atan(tag.pose_R[2,1]/tag.pose_R[2,2])/math.pi*180)

	    #print(tag.pose_t[1][0])
	    #a = tag.pose_R[2][1]
	    #a = a*a
            #b = tag.pose_R[2][2]*tag.pose_R[2][2]
	    #thetay = math.atan2(-tag.pose_R[2][0],math.sqrt(a+b))
	    #print(thetay)
            center = [320,240]
            frame = plotPoint(frame, tag.center, CENTER_COLOR)
            frame = plotPoint(frame, center, CENTER_COLOR)
            if tag.center[0] < 320 - threshold:
                print("turn left")
            elif tag. center[0] > 320 + threshold:
                print("turn right")
            else:
                print("go straight")
            #frame = plotText(frame, tag.center, CENTER_COLOR, tag.pose_R)
            #print("phi = ", math.asin(-tag.pose_R[2,0])/math.pi*180)
            #print("theta = ", math.atan(tag.pose_R[2,1]/tag.pose_R[2,2])/math.pi*180)
            #print("psi = ", math.atan(tag.pose_R[1,0]/tag.pose_R[0,0])/math.pi*180)
            for corner in tag.corners:
                frame = plotPoint(frame, corner, CORNER_COLOR)

    cv2.imshow('frame', frame)

    key = cv2.waitKey(100) #ms
	# terminate the loop if the 'Return' key is hit
    if key == 13:
        looping = False

cv2.destroyAllWindows()


