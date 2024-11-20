from djitellopy import tello
import KeyPressModule as kp
import numpy as np
import cv2
import math
import os
from time import sleep, time

######## PARAMETERS ###########
fSpeed = 117 / 10  # Forward Speed in cm/s (15cm/s)
aSpeed = 360 / 10  # Angular Speed Degrees/s (50d/s)
interval = 0.25
dInterval = fSpeed * interval
aInterval = aSpeed * interval

fps = 30  # Desired frames per second
frame_time = 1 / fps  # Time per frame

###############################################
x, y = 500, 500
a = 0
yaw = 0
points = [(0, 0), (0, 0)]

kp.init()

# Connect to the drone
me = tello.Tello()
me.connect()
print("Battery level:", me.get_battery())

# Start the camera stream
me.streamon()

# Output directory for videos
output_dir = r"C:\Users\EXCALIBUR\Desktop\drones"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Initialize video writer
video_index = 1
video_path = os.path.join(output_dir, f"test_{video_index}.mp4")
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video_writer = None

# Initialize ORB detector
orb = cv2.ORB_create()

def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 15
    aspeed = 50
    global x, y, yaw, a, video_writer

    d = 0
    if kp.getKey("LEFT"):
        lr = -speed
        d = dInterval
        a = -180
    elif kp.getKey("RIGHT"):
        lr = speed
        d = -dInterval
        a = 180
    if kp.getKey("UP"):
        fb = speed
        d = dInterval
        a = 270
    elif kp.getKey("DOWN"):
        fb = -speed
        d = -dInterval
        a = -90
    if kp.getKey("w"):
        ud = speed
    elif kp.getKey("s"):
        ud = -speed
    if kp.getKey("a"):
        yv = -aspeed
        yaw -= aInterval
    elif kp.getKey("d"):
        yv = aspeed
        yaw += aInterval
    if kp.getKey("q"):
        # Land the drone, release resources, and stop the program
        me.land()
        if video_writer:
            video_writer.release()
        cv2.destroyAllWindows()
        me.streamoff()
        exit()
    if kp.getKey("e"):
        me.takeoff()

    sleep(interval)
    a += yaw
    x += int(d * math.cos(math.radians(a)))
    y += int(d * math.sin(math.radians(a)))

    return [lr, fb, ud, yv, x, y]

def drawPoints(img, points):
    for point in points:
        cv2.circle(img, point, 5, (0, 0, 255), cv2.FILLED)
    cv2.circle(img, points[-1], 8, (0, 255, 0), cv2.FILLED)
    cv2.putText(img, f'({(points[-1][0] - 500) / 100},{(points[-1][1] - 500) / 100})m',
                (points[-1][0] + 10, points[-1][1] + 30), cv2.FONT_HERSHEY_PLAIN, 1,
                (255, 0, 255), 1)

previous_time = time()

while True:
    current_time = time()
    elapsed_time = current_time - previous_time

    if elapsed_time >= frame_time:
        previous_time = current_time

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

        # Initialize video writer on the first frame
        if video_writer is None:
            video_writer = cv2.VideoWriter(video_path, fourcc, fps, (640, 480))

        # Write the processed frame to the video file
        video_writer.write(frame_with_keypoints)

        # Create a blank image for the trajectory map
        img = np.zeros((1000, 1000, 3), np.uint8)

        # Update trajectory points
        if points[-1][0] != vals[4] or points[-1][1] != vals[5]:
            points.append((vals[4], vals[5]))

        # Draw the trajectory map
        drawPoints(img, points)

        # Display the video and trajectory map
        cv2.imshow("Drone Camera with ORB Features", frame_with_keypoints)
        cv2.imshow("Trajectory Map", img)

    # Exit on pressing 'ESC'
    if cv2.waitKey(1) & 0xFF == 27:
        me.land()
        if video_writer:
            video_writer.release()
        cv2.destroyAllWindows()
        me.streamoff()
        break
