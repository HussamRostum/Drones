import cv2
import numpy as np

# Load the image
image_path = 'r.png'
image = cv2.imread(image_path)

# Initialize ORB detector
orb = cv2.ORB_create(nfeatures=100)

# Convert the image to grayscale (ORB works on single-channel images)
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Detect keypoints and compute descriptors
keypoints, descriptors = orb.detectAndCompute(gray_image, None)

# Draw keypoints on the original image
image_with_keypoints = cv2.drawKeypoints(
    image, keypoints, None, color=(0, 255, 0), flags=cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)

# Resize images for display
medium_size = (640, 480)
image = cv2.resize(image, medium_size)
image_with_keypoints = cv2.resize(image_with_keypoints, medium_size)

# Combine images side-by-side
combined_image = np.hstack((image, image_with_keypoints))

# Display the combined image
cv2.imshow('Original and ORB Features', combined_image)

# Wait until a key is pressed and close the image window
cv2.waitKey(0)
cv2.destroyAllWindows()
