#!/usr/bin/env python3

import rospy
import argparse
import telnetlib
import websocket
from grid_transmitter.msg import PwmCombined


class Transmitter:
    def __init__(self, args):
        rospy.init_node('grid_transmitter_{}'.format(args.namespace))
        if args.topic:
            rospy.Subscriber(args.topic, PwmCombined, self.callback)
        else:
            rospy.Subscriber('grid_robot_{}/pwm'.format(args.namespace),
                             PwmCombined, self.callback)
        self.telnet_flag = args.telnet

        default_ip = ['192.168.0.106', '192.168.0.107',
                      '192.168.0.108', '192.168.0.109']
        ip = args.ip if args.ip != '' else default_ip[args.namespace-1]

        if args.telnet:
            self.telnet = telnetlib.Telnet(ip, args.port)
            rospy.loginfo("Telnet connected to %s:%s", ip, args.port)
        else:
            self.websocket = websocket.WebSocket()
            self.websocket.connect("ws://{}:{}".format(ip, args.port))
            rospy.loginfo("WebSocket connected to %s:%s", ip, args.port)

    def __del__(self):
        if self.telnet_flag:
            self.telnet.close()
        else:
            self.websocket.close()

    def callback(self, data):
        msg = "{},{},{}\r".format(data.left, data.right, data.servo)
        if self.telnet_flag:
            self.telnet.write(msg.encode("ascii"))
        else:
            self.websocket.send(msg)
            print(msg)
            rospy.loginfo(self.websocket.recv())


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('namespace', type=int, default=0,
                        help='robot namespace, default: 0')
    parser.add_argument('--ip', type=str, default='',
                        help='robot ip address, default: ""')
    parser.add_argument('--topic', type=str,
                        help='pwm ros topic, default: None')
    parser.add_argument('--port', type=int, default=8888,
                        help='port, default: 8888')
    parser.add_argument('--telnet', action='store_true',
                        help='use telnet comm')
    args = parser.parse_args()
    assert 1 <= args.namespace <= 4

    transmitter = Transmitter(args)
    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.loginfo(e)
