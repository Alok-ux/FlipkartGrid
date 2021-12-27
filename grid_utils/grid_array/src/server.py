#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import argparse
from cv_bridge import CvBridge, CvBridgeError
from grid_array.srv import GridArray, GridArrayResponse

def nothing(data):
    pass

class TrackDetector:
    def __init__(self, args):
        # rospy.init_node(args.srv)
        # self.bridge = CvBridge()
        # rospy.Service(args.srv, GridArray, self.handle_goal_response)
        # rospy.loginfo("Server %s started", args.srv)

        cv2.namedWindow('thresh', cv2.WINDOW_NORMAL)
        cv2.createTrackbar("kernel", "thresh", 1, 5, nothing)
        cv2.createTrackbar("iterations", "thresh", 1, 10, nothing)
        cv2.createTrackbar("arclen", "thresh", 0, 255, nothing)
        cv2.createTrackbar("ar_min", "thresh", 0, 255, nothing)
        cv2.createTrackbar("ar_max", "thresh", 0, 255, nothing)
        while True:
            image = cv2.imread("/home/lucifer/Desktop/only_arena.png")
            self.process(image)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def handle_goal_response(self, req):
        try:
            image = self.bridge.imgmsg_to_cv2(req.image)
            self.process(image)
            cv2.imshow('image', image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            # return GoalArrayResponse()
        except CvBridgeError as e:
            rospy.loginfo(e)

    def process(self, image):
        kernel_size = cv2.getTrackbarPos("kernel", "thresh")
        iterations = cv2.getTrackbarPos("iterations", "thresh")
        # l_v = cv2.getTrackbarPos("Lower - V", "thresh")
        # u_h = cv2.getTrackbarPos("Upper - H", "thresh")
        # u_s = cv2.getTrackbarPos("Upper - S", "thresh")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        erode = cv2.erode(thresh, kernel=np.ones((kernel_size,kernel_size), np.uint8), iterations=iterations)
        # thresh = cv2.inRange(image, (l_h, l_s, l_v), (u_h, u_s, u_v))

        contours, _ = cv2.findContours(erode, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)

            if len(approx) == 4:
                (x, y, w, h) = cv2.boundingRect(approx)
                ar = w / float(h)
                cv2.drawContours(image, [cnt], 0, (0, 0, 255), 5)

                if 0.95 <= ar <= 1.05:
                    M = cv2.moments(cnt)
                    x = int(M['m10']/(M['m00'] + 1e-5))
                    y = int(M['m01']/(M['m00'] + 1e-5))


        hstack = np.hstack([image, cv2.cvtColor(erode, cv2.COLOR_GRAY2BGR)])
        cv2.imshow('thresh', hstack)

if __name__ == "__main__":
    parser = argparse.ArgumentParser('track_detector')
    parser.add_argument('--srv', type=str, default='get_grid_array',
                        help='server name, default: get_grid_array')
    args = parser.parse_args()
    detector = TrackDetector(args)
    # try:
    #     rospy.spin()
    # except ROSInterruptException as e:
    #     rospy.loginfo(e)
