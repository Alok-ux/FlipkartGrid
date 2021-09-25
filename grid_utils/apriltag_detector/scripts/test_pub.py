#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
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

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass