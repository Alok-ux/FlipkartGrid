#!/usr/bin/env python3

import cv2
import rospy
import argparse

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CameraDriver:
    def __init__(self, args):
        rospy.init_node('camera_driver')
        self.pub = rospy.Publisher(args.topic, Image, queue_size=1)
        self.bridge = CvBridge()
        self.source = int(args.source) if args.source.isdigit() else args.source

    def publish(self):
        cap = cv2.VideoCapture(self.source)
        while cap.isOpened() and not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                break
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                self.pub.publish(msg)
            except CvBridgeError as e:
                print(e)


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Camera Driver')
    parser.add_argument('source', nargs='?', type=str, default='0')
    parser.add_argument('-t', '--topic', type=str, default='/overhead_camera/image_raw')
    args = parser.parse_args()

    camera_driver = CameraDriver(args)
    try:
        camera_driver.publish()
    except ROSInterruptException as e:
        print(e)
