#!/usr/bin/env python3

import cv2
import rospy
import argparse
import apriltag
from std_msgs.msg import Int32,String
import time
import math



list= [[304,112],[304,361],[]]

class CameraDriver:
    def __init__(self, args):
        rospy.init_node('camera_driver')
        self.pub1 = rospy.Publisher("apriltag_centre1",String,queue_size = 1)
        self.last = time.time()
        self.debug = args.debug
        self.source = int(args.source) if args.source.isdigit() else args.source

    def publish(self):
        cap = cv2.VideoCapture(self.source)
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        print(cap.get(cv2.CAP_PROP_FPS))

        while cap.isOpened() and not rospy.is_shutdown():
            ret, frame = cap.read()
            now=time.time()
            now=now-self.last
            self.last=self.last+now
            if not ret:
                break
            try:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                options = apriltag.DetectorOptions('tag36h11')
                detector = apriltag.Detector(options)
                results = detector.detect(gray)
                pStr="FPS: {} delay : {}ms ".format(int(1/now),int(1000*now))
                for i in results:
                    pta , ptb ,ptc ,ptd = i.corners
                    cn = i.center
                    pta = (int(pta[0]),int(pta[1]))
                    ptb = (int(ptb[0]),int(ptb[1]))
                    ptc = (int(ptc[0]),int(ptc[1]))
                    ptd = (int(ptd[0]),int(ptd[1]))
                    pte = (int((pta[0]+ptb[0])/2),int((pta[1]+ptb[1])/2))
                    cn = (int(cn[0]),int(cn[1]))
                    pStr+= "Center []: {},{} ".format(cn[0],cn[1])
                    cv2.rectangle(frame,pta,ptc,(0,255,0),2)
                    cv2.circle(frame,cn,3,(0,0,255),-1)
                    cv2.circle(frame,pte,3,(0,0,255),-1)
                    theta= math.degrees(math.atan2(pte[1]-cn[1] , pte[0]-cn[0]))
                    ptr = str(cn[0])+" "+str(cn[1])+" "+str(theta)
                    print(ptr)
                    self.pub1.publish(ptr)
                print(pStr)
                if self.debug:
                    frame = cv2.line(frame, (list[1][0],list[1][1]), (list[0][0],list[0][1]), (0,255,0), 2)
                    cv2.imshow("Frame", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        pass

            except Exception as e:
                print(e)



if __name__ == '__main__':
    parser = argparse.ArgumentParser('Camera Driver')
    parser.add_argument('source', nargs='?', type=str, default='0')
    parser.add_argument('-d', '--debug', type=bool, default='true')
    parser.add_argument('-t', '--topic', type=str, default='/overhead_camera/image_raw')
    args = parser.parse_args()

    camera_driver = CameraDriver(args)
    try:
        camera_driver.publish()
    except Exception as e:
        print(e)
