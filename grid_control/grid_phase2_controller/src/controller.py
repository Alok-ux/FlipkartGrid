#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32
from geometry_msgs import Twist
import math

kp_ang = 0.005
ki_ang = 0
kd_ang = 0.001
kp_dist = 0.005
ki_dist = 0
kd_dist = 0.001
prev_dist = 0
prev_ang = 0
I_dist = 0
I_ang = 0
act_dif_dist = 5


class motion():
    def __init__(self):
        rospy.init_node("bot_motion", anonymous=True)
        self.sub1 = rospy.Subscriber(
            "apriltag_centre1", String, self.callback_pose)
        # self.sub2= rospy.Subscriber("Co-ordinates from client",String , self.callback_co_ord)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.msg = Twist()

    def callback_pose(self, data):
        pose = list(data.data.split(" "))
        self.center = [pose[0], pose[1]]
        self.ang = pose[2]

    # def callback_co_ord(self,data):
    #     self.destination = list(data.data.split(" "))

    def move(self):
        x1 = 304-self.center[0]
        y1 = 361-self.center[1]
        dist = (x1**2+y1**2)**0.5
        deg1 = math.degrees(math.atan2(y1, x1))
        ang = deg1-self.ang
        if ang < -180:
            ang += 360
        if ang > 180:
            ang -= 360
        final_speed = pid(dist, ang)
        vel = 100 + final_speed[0]*5
        if dist < 5:
            self.msg.linear.x = 0
            self.msg.angular.z = 0
        else:
            self.msg.linear.x = vel
            self.msg.angular.z = final_speed[1]
        self.pub.publish(self.msg)

    def pid(self, dist, ang):
        global kp_ang, ki_ang, kd_ang, kp_dist, ki_dist, kd_dist, prev_dist, prev_ang, I_dist, I_ang
        err_ang = ang
        err_dist = dist
        diff_dist = err_dist - I_dist
        lim_dist = act_dif_dist - diff_dist
        diff_ang = err_ang - I_ang
        final_dist = kp_dist*lim_dist + ki_dist*lim_dist
        final_ang = kp_ang*err_ang + ki_ang*err_ang + kd_ang*diff_ang
        I_dist = err_dist
        I_ang = err_ang
        speed = [final_dist, final_ang]

        return speed


if __name__ == '__main__':
    obj = motion()
    try:
        rospy.spin()
    except Exception as e:
        print(e)
