#!/usr/bin/env python3

# import sys
import math
import rospy
import argparse
import apriltag
import cv2 as cv
from camera_driver.msg import GridPose, GridPoseArray
from cv_bridge import CvBridge


class PoseCamDriver:
    def __init__(self, source, tag_class):
        rospy.init_node('cam_driver')
        options = apriltag.DetectorOptions('tag{}'.format(tag_class))
        self.detector = apriltag.Detector(options)
        self.pub = rospy.Publisher(
            'grid_robot/poses', GridPoseArray, queue_size=1)
        self.cv_bridge = CvBridge()
        self.rate = rospy.Rate(2000)  # 10Hz
        self.source = int(source) if source.isdigit() else source
        rospy.loginfo("Camera Driver Started")

    def read(self):
        cap = cv.VideoCapture(self.source)
        while not rospy.is_shutdown() and cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                cap.open(self.source)
                continue

            # cap.set(3, 640)
            # cap.set(4, 480)
            # frame = frame[1:479, 41:593]
            # frame = frame[103:452, 113:564]
            # frame = frame[5:480, 40:593]
            # frame = frame[23:473, 43:575]  # DBA A120
            # frame = frame[160:455, 245:390]  # A124

            # for i in range(1, 5):
            #     for j in range(1, 9):
            #         cv.putText(frame, f'{i-1}, {j-1}', (36*i-18, 36*j-18), cv.FONT_HERSHEY_SIMPLEX, 0.25, (0, 0, 255), 1)

            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            results = self.detector.detect(gray)

            array = GridPoseArray()

            for result in results:
                xc, yc = result.center
                (x1, y1), (x2, y2) = result.corners[0], result.corners[1]
                xm, ym = (x1 + x2) / 2, (y1 + y2) / 2

                msg = GridPose(id=result.tag_id, x=xc, y=yc)
                msg.theta = math.atan2((ym-yc), (xm-xc))

                cv.arrowedLine(frame, (int(xc), int(yc)),
                               (int(xm), int(ym)), (0, 255, 0), 2)
                array.poses.append(msg)

            array.image = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            print(array.poses)
            self.pub.publish(array)
            cv.imshow('frame', frame)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('source', nargs='?', type=str, default='0')
    parser.add_argument('tag_class', nargs='?', type=str, default='36h11')
    args = parser.parse_args()
    try:
        frame_feed = PoseCamDriver(args.source, args.tag_class)
        frame_feed.read()
    except rospy.ROSInterruptException as e:
        print(e)
