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
        self.loop = args.loop
        rospy.loginfo("Camera driver started at %s", args.source)
        self.rate = rospy.Rate(60)

    def publish(self):
        cap = cv2.VideoCapture(self.source)
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        rospy.loginfo("FPS: %s", cap.get(cv2.CAP_PROP_FPS))

        while cap.isOpened() and not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                if self.loop:
                    cap.open(self.source)
                    continue
                else:
                    break
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                self.pub.publish(msg)
                self.rate.sleep()
            except CvBridgeError as e:
                print(e)


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Camera Driver')
    parser.add_argument('source', nargs='?', type=str, default='2',
                        help='source device/file location, default: 2')
    parser.add_argument('--topic', type=str,
                        default='/overhead_camera/image_raw',
                        help='camera topic, default: /overhead_camera/image_raw')
    parser.add_argument('--width', type=int, default=800,
                        help='set video width, default: 800')
    parser.add_argument('--height', type=int, default=600,
                        help='set video height, default: 600')
    parser.add_argument('--loop', action='store_true',
                        help='set video file in loop')
    args = parser.parse_args()
    camera_driver = CameraDriver(args)
    try:
        camera_driver.publish()
    except rospy.ROSInterruptException as e:
        print(e)
