from djitellopy import tello
import KeyPressModule as kp
import cv2
import os
from time import sleep

# Initialize keyboard control module
kp.init()

# Connect to the drone
me = tello.Tello()
me.connect()
print("Battery level:", me.get_battery())

# Start the camera stream
me.streamon()

# Directory to save videos
output_dir = r"C:\Users\EXCALIBUR\Desktop\drones"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Generate sequential video names
video_index = 1
video_path = os.path.join(output_dir, f"test_{video_index}.mp4")

# Define video writer (will initialize after first frame capture)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video_writer = None
fps = 30  # Frames per second


def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 50
    global video_index, video_path, video_writer

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
        # Land the drone, stop video recording, and exit the program
        me.land()
        if video_writer:
            video_writer.release()
        cv2.destroyAllWindows()
        me.streamoff()
        exit()
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

    # Initialize video writer on the first frame
    if video_writer is None:
        video_writer = cv2.VideoWriter(video_path, fourcc, fps, (640, 480))

    # Write the processed frame to the video file
    video_writer.write(frame_with_keypoints)

    # Display the processed video frame
    cv2.imshow("Drone Camera with ORB Features", frame_with_keypoints)

    # Exit on pressing 'ESC'
    if cv2.waitKey(1) & 0xFF == 27:  # Press ESC to break
        me.land()
        if video_writer:
            video_writer.release()
        cv2.destroyAllWindows()
        me.streamoff()
        break

    sleep(0.05)
