#!/usr/bin/env python

import socket
import cv2
import numpy as np
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ip_add = "192.168.134.30"
port = 4210

def forward(speed):
    print("forward")
    left_speed = speed
    right_speed = speed
    servo_angle = 0
    msg = "{},{},{}\r".format(left_speed, right_speed, servo_angle)
    sock.sendto(str.encode(msg), (ip_add, port))

def left(speed):
    print("left")
    left_speed = speed
    right_speed = speed

def right(speed):
    print("right")
    left_speed = speed
    right_speed = speed

def back(speed):
    print("back")
    left_speed = speed
    right_speed = speed

def stop(speed):
    print("stop")
    left_speed = speed
    right_speed = speed
    servo_angle = 0
    msg = "{},{},{}\r".format(left_speed, right_speed, servo_angle)
    sock.sendto(str.encode(msg), (ip_add, port))

while True:
    blank = np.zeros((200,200,3), np.uint8)
    cv2.imshow("window", blank)
    key_pressed = cv2.waitKey(0)

    if key_pressed == 255:
        stop(0)

    elif key_pressed == 82:
        forward(255)

    elif key_pressed == 83:
        right(None)

    elif key_pressed == 84:
        back(None)

    elif key_pressed == 85:
        left(None)

    elif key_pressed == 32:
        stop(0)

    elif key_pressed == ord('q'):
        break

    print(key_pressed)

cv2.destroyAllWindows()