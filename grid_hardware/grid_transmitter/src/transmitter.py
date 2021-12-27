#!/usr/bin/env python3

import rospy
import argparse
import telnetlib
from differential_drive.msg import DiffRPM

class Transmitter:
    def __init__(self, args):
        rospy.init_node('transmitter')
        rospy.Subscriber(args.topic, DiffRPM, self.callback)
        self.Host= args.ip
        rospy.loginfo("Telnet connected to %s:%s", args.ip, args.port)

    def callback(self, data):
        msg = "{},{},0\r".format(int(data.left_rpm*10), int(data.right_rpm*10))
        tn = telnetlib.Telnet(self.Host,8888)
        print(msg)
        tn.write(msg.encode("ascii"))
        tn.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', type=str, default='/grid_robot/rpm')
    parser.add_argument('--ip', type=str, default='192.168.29.105')
    parser.add_argument('--port', type=int, default=8888)
    args = parser.parse_args()

    transmitter = Transmitter(args)
    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
