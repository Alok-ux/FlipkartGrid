#!/usr/bin/env python3

import rospy
import argparse
import telnetlib
import websocket
from differential_drive.msg import DiffRPM

class Transmitter:
    def __init__(self, args):
        rospy.init_node('transmitter')
        rospy.Subscriber(args.topic, DiffRPM, self.callback)
        self.telnet_flag = args.telnet

        if args.telnet:
            self.telnet = telnetlib.Telnet(args.ip, args.port)
            rospy.loginfo("Telnet connected to %s:%s", args.ip, args.port)
        else:
            self.websocket = websocket.WebSocket()
            self.websocket.connect("ws://{}:{}".format(args.ip, args.port))
            rospy.loginfo("WebSocket connected to %s:%s", args.ip, args.port)

    def __del__(self):
        if self.telnet_flag:
            self.telnet.close()
        else:
            self.websocket.close()

    def callback(self, data):
        msg = "{},{},0\r".format(int(data.left_rpm*10), int(data.right_rpm*10))
        if self.telnet_flag:
            self.telnet.write(msg.encode("ascii"))
        else:
            self.websocket.send(msg)
            rospy.loginfo(self.websocket.recv())


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('ip', nargs='?', type=str, help='robot ip address')
    parser.add_argument('--topic', type=str, default='/grid_robot/rpm',
                        help='rpm ros topic, default:/grid_robot/rpm')
    parser.add_argument('--port', type=int, default=8888,
                        help='port, default:8888')
    parser.add_argument('--telnet', action='store_true', help='use telnet comm')
    args = parser.parse_args()

    if args.ip:
        transmitter = Transmitter(args)
    else:
        rospy.loginfo("ip is not provided")
    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.loginfo(e)
