import cv2
from apriltag import apriltag

def detect_apriltag(frame):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Create the detector
    detector = apriltag("tag36h11")

    # Detect AprilTags in the frame
    results = detector.detect(gray)

    if len(results) > 0:
        # Assuming there is only one AprilTag in the frame
        april_tag = results[0]

        # Get the center coordinates of the AprilTag
        center_x = april_tag.center[0]
        frame_width = frame.shape[1]
        
        if center_x < frame_width / 2:
            return "Left side"
        else:
            return "Right side"
    else:
        return "No AprilTag found"

# Open the webcam
video_capture = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    ret, frame = video_capture.read()

    # Detect the AprilTag and determine its location
    location = detect_apriltag(frame)

    # Display the result on the frame
    cv2.putText(frame, f"AprilTag location: {location}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Display the frame
    cv2.imshow("AprilTag Detection", frame)

    # Check for the 'q' key to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close any open windows
video_capture.release()
cv2.destroyAllWindows()
