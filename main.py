from djitellopy import Tello
from time import sleep
import cv2

me= Tello()
me.connect()
print("the battary is ",me.get_battery())

me.takeoff()
me.send_rc_control(0, 0, 50, 0)
sleep(2)
me.send_rc_control(0, 0, 0, 0)
sleep(2)
me.send_rc_control(0, 0, 0, 0)
sleep(2)
me.send_rc_control(0, 0, 0, 0)
me.land()