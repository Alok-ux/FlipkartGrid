#!/usr/bin/env python3

import cv2
import rospy
import argparse
from cv_bridge import CvBridge, CvBridgeError
from grid_array.srv import GridArray, GridArrayResponse


class TrackDetector:
    def __init__(self, args):
        rospy.init_node('get_grid_array')
        self.bridge = CvBridge()
        rospy.Service('get_grid_array', GridArray, self.handle_goal_response)

    def handle_goal_response(self, req):
        try:
            self.image = self.bridge.imgmsg_to_cv2(req, 'bgr8')
            return GoalArrayResponse(req.a + req.b)
        except CvBridgeError as e:
            rospy.loginfo(e)



if __name__ == "__main__":
    parser = argparse.ArgumentParser('track_detector')
    parser.add_argument('tag_id', type=int, default=1,
                        help='robot tag id, default: 1')
    args = parser.parse_args()
    detector = TrackDetector(args)
    try:
        rospy.spin()
    except ROSInterruptException as e:
        rospy.loginfo(e)
