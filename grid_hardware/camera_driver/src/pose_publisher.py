#!/usr/bin/env python

# import sys
import math
import rospy
import argparse
import apriltag
import cv2 as cv
from camera_driver.msg import GridPose, GridPoseArray


class PoseCamDriver:
    def __init__(self, source, tag_class):
        rospy.init_node('cam_driver')
        options = apriltag.DetectorOptions('tag{}'.format(tag_class))
        self.detector = apriltag.Detector(options)
        self.pub = rospy.Publisher(
            'grid_robot/poses', GridPoseArray, queue_size=1)
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
            frame = frame[1:479, 41:593]

            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            results = self.detector.detect(gray)

            array = GridPoseArray()

            for result in results:
                xc, yc = result.center
                (x1, y1), (x2, y2) = result.corners[0], result.corners[1]
                xm, ym = (x1 + x2) / 2, (y1 + y2) / 2
                self.i = int(math.floor(xc/34))
                self.j = int(math.floor(yc/34))
                msg = GridPose(id=result.tag_id,i = self.i,j = self.j, x=xc, y=yc)
                msg.theta = math.atan2((yc-ym), (xc-xm))

                #TODO: map pixel to real world coordinates (sanjeet)

                cv.arrowedLine(frame, (int(xc), int(yc)),
                               (int(xm), int(ym)), (0, 255, 0), 2)
                array.poses.append(msg)

            self.pub.publish(array)
            cv.imshow('frame', frame)
            if cv.waitKey(30) & 0xFF == ord('q'):
                break
        cap.release()
        cv.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('source', nargs='?', type=str, default=0)
    parser.add_argument('tag_class', nargs='?', type=str, default='36h11')
    args = parser.parse_args()
    try:
        frame_feed = PoseCamDriver(args.source, args.tag_class)
        frame_feed.read()
    except rospy.ROSInterruptException as e:
        print(e)
