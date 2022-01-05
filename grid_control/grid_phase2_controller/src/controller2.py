#!/usr/bin/env python3

import math
import rospy
import cv2
import numpy as np
from camera_driver.msg import GridPoseArray
from grid_transmitter.msg import PwmCombined
from std_msgs.msg import Float32


class Motion():
    def __init__(self):
        rospy.init_node("bot_motion")
        self.lis = [(145.59, 244.01), (293.6, 241.07),
                    (287.15, 94.84), (141.44, 95.18)]
        self.id = 0
        self.pose = 0
        self.kp, self.ki, self.kd = 1, 0.0, 0.0
        self.intg, self.max_intg, self.lastError = 0.0, 1.0, 0.0
        self.base_speed = 100
        self.i = 0
        self.msg = PwmCombined()
        self.pub = rospy.Publisher(
            '/grid_robot/pwm_1', PwmCombined, queue_size=10)
        self.pub_error = rospy.Publisher('/error', Float32, queue_size=10)
        rospy.Subscriber('grid_robot/poses', GridPoseArray, self.callback_pose)

    def callback_pose(self, msg):
        self.pose = [pose for pose in msg.poses if pose.id == self.id]
        self.move()

    def move(self):
        if len(self.pose):
            # x1, x2, y1, y2 = self.pose[0].x, 379.52, self.pose[0].y, 43.52

            x2, y2 = self.lis[self.i]
            x1, y1 = self.pose[0].x, self.pose[0].y
            self.target_angle = math.degrees(math.atan2(y2-y1, x2-x1))
            self.robot_angle = math.degrees(self.pose[0].theta)
            dist = ((x2-x1)**2+(y2-y1)**2)**0.5

            if dist <= 40 and self.i < 3:
                self.msg.left = 0
                self.msg.right = 0
                self.pub.publish(self.msg)
                self.i = self.i+1
                x2, y2 = self.lis[self.i]
                self.target_angle = math.degrees(math.atan2(y2-y1, x2-x1))
                error = self.target_angle - self.robot_angle
                self.orient(error)

            elif dist <= 40 and self.i >= 3:
                self.msg.left = 0
                self.msg.right = 0
                self.pub.publish(self.msg)

            else:
                print("entering loop")
                error = self.target_angle - self.robot_angle
                if error > 180:
                    error -= 360
                if error < -180:
                    error += 360
                balance = self.pid(error)
                self.msg.left = int(self.base_speed + balance)
                self.msg.right = int(self.base_speed - balance)
                self.pub.publish(self.msg)

            # error = self.target_angle - self.robot_angle
            # if error > 180:
            #     error -= 360
            # if error < -180:
            #     error += 360

            # image = np.ones((480, 640), dtype=np.uint8) * 0
            # cv2.arrowedLine(image, (int(x1), int(y1)),
            #                 (int(x2), int(y2)), (255, 0, 0), 2)
            # cv2.imshow('image', image)
            # cv2.waitKey(1)
            # print(robot_angle, target_angle, error)
            # balance = self.pid(error)
            #
            # print(self.base_speed, balance)
            #
            # self.msg.left = int(self.base_speed + balance)
            # self.msg.right = int(self.base_speed - balance)
            # self.pub.publish(self.msg)
            # self.pub_error.publish(error)

    def pid(self, error):
        prop = error
        self.intg += error
        if self.intg >= self.max_intg:
            self.intg = self.max_intg
        if self.intg <= -self.max_intg:
            self.intg = -self.max_intg
        diff = error - self.lastError
        self.lastError = error
        return self.kp * prop + self.ki * self.intg + self.kd * diff

    def orient(self, error):
        if error > 20 or error < -20:
            balance = self.pid(error)
            self.msg.left = int(balance)
            self.msg.right = int(-balance)
            self.pub.publish(self.msg)


if __name__ == '__main__':
    obj = Motion()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)