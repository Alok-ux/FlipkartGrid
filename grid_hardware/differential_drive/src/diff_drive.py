#!/usr/bin/env python3

import rospy
import argparse
from geometry_msgs.msg import Twist
from differential_drive.msg import DiffRPM

class DifferentialDrive:
    def __init__(self, args):
        rospy.init_node('differential_drive')
        self.sub = rospy.Subscriber("cmd_vel", Twist, self.callback)
        self.pub = rospy.Publisher(args.rpm, DiffRPM, queue_size=10)
        self.wheel_separation = args.separation
        self.wheel_diameter= args.diameter

    def callback(self, data):
        v = data.linear.x
        w = data.angular.z

        msg = DiffRPM()
        msg.right_rpm = (2 * v + w * self.wheel_separation) * 60 / (2 * self.wheel_diameter * 3.14)
        msg.left_rpm =  (2 * v - w * self.wheel_separation) * 60 / (2 * self.wheel_diameter * 3.14)
        rospy.loginfo(msg)
        self.pub.publish(msg)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--cmd_vel', type=str, default='/grid_robot/cmd_vel',
                        help='cmd_vel topic, default:/grid_robot/cmd_vel')
    parser.add_argument('--rpm', type=str, default='/grid_robot/rpm',
                        help='rpm topic, default:/grid_robot/rpm')
    parser.add_argument('-s', '--separation', type=float, default=10,
                        help='wheel separation, default=10')
    parser.add_argument('-d', '--diameter', type=float, default=10,
                        help='wheel diameter, default=10')
    args = parser.parse_args()

    differential_drive = DifferentialDrive(args)
    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
