from djitellopy import tello
import KeyPressModule as kp
import cv2
import numpy as np
from time import sleep

# Initialize the keyboard control module
kp.init()

# Connect to the drone
me = tello.Tello()
me.connect()
print("Battery level:", me.get_battery())

# Start the camera stream
me.streamon()


def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 50
    if kp.getKey("LEFT"):
        lr = -speed
    elif kp.getKey("RIGHT"):
        lr = speed
    if kp.getKey("UP"):
        fb = speed
    elif kp.getKey("DOWN"):
        fb = -speed
    if kp.getKey("w"):
        ud = speed
    elif kp.getKey("s"):
        ud = -speed
    if kp.getKey("a"):
        yv = -speed
    elif kp.getKey("d"):
        yv = speed
    if kp.getKey("q"):
        me.land()
        sleep(3)
    if kp.getKey("e"):
        me.takeoff()
    return [lr, fb, ud, yv]


# Initialize ORB detector
orb = cv2.ORB_create()

while True:
    # Capture keyboard inputs for drone control
    vals = getKeyboardInput()
    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])

    # Get the video frame from the drone's camera
    frame = me.get_frame_read().frame
    frame = cv2.resize(frame, (640, 480))  # Resize frame for consistent processing

    # Apply ORB feature detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
    keypoints, descriptors = orb.detectAndCompute(gray, None)  # Detect features
    frame_with_keypoints = cv2.drawKeypoints(
        frame, keypoints, None, color=(0, 255, 0), flags=0
    )  # Draw keypoints on the frame

    # Display the processed video frame
    cv2.imshow("Drone Camera with ORB Features", frame_with_keypoints)

    # Exit on pressing 'ESC'
    if cv2.waitKey(1) & 0xFF == 27:  # Press ESC to break
        break

    sleep(0.05)

# Cleanup
cv2.destroyAllWindows()
me.streamoff()
