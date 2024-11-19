from djitellopy import Tello
from time import sleep
import cv2


me= Tello()
me.connect()
print("the battary is ",me.get_battery())

me.streamon()
while True:
    img = me.get_frame_read().frame
    img = cv2.resize(img,(360,240))
    cv2.imshow('Image',img)
    cv2.waitKey(1)