#!/usr/bin/env python3

import cv2
import rospy
import argparse
import apriltag
from sensor_msgs.msg import Image
from std_msgs.msg import Int32,String,Int32MultiArray, MultiArrayLayout,MultiArrayDimension
from cv_bridge import CvBridge, CvBridgeError
# from camera_driver.msg import junction

list1= [[300,340],[301,110],[340,70], [580,70]]

class CameraDriver:
    def __init__(self, args):
        rospy.init_node('camera_driver')
        # self.pub = rospy.Publisher(args.topic, Image, queue_size=1)
        self.pub1 = rospy.Publisher("apriltag_centre1",String,queue_size = 1)
        # self.pub2 = rospy.Publisher("junctions",Int32MultiArray,queue_size = 1)
        # self.pub2 = rospy.Publisher(args.rpm,junction, queue_size=1)
        self.bridge = CvBridge()
        self.source = int(args.source) if args.source.isdigit() else args.source

    def publish(self):
        cap = cv2.VideoCapture(self.source, cv2.CAP_V4L)
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        print(cap.get(cv2.CAP_PROP_FPS))

        while cap.isOpened() and not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                break
            try:
                # frame=self.bridge.imgmsg_to_cv2(data, 'bgr8')
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                options = apriltag.DetectorOptions(families ='tag16h5')
                detector = apriltag.Detector(options)
                results = detector.detect(gray)
                ptr=""
                # print(results)
                for i in results:
                    if i.tag_id == 2:
                        pta , ptb ,ptc ,ptd = i.corners
                        cn = i.center
                        pta = (int(pta[0]),int(pta[1]))
                        ptb = (int(ptb[0]),int(ptb[1]))
                        ptc = (int(ptc[0]),int(ptc[1]))
                        ptd = (int(ptd[0]),int(ptd[1]))
                        pte = (int((pta[0]+ptb[0])/2),int((pta[1]+ptb[1])/2))
                        cn = (int(cn[0]),int(cn[1]))
                        print("center: ",cn[0]," ",cn[1])
                        cv2.line(frame,cn,pte,(255,255,0),2)
                        cv2.circle(frame,cn,3,(0,0,255),-1)
                        cv2.circle(frame,pte,3,(0,0,255),-1)
                        ptr = str(cn[0])+" "+str(cn[1])+" "+str(pte[0])+" "+str(pte[1])
                        # cv2.line(frame, (list[1][0],list[1][1]), (list[0][0],list[0][1]), (255,255,0), 2)
                        self.pub1.publish(ptr)
                # cv2.imshow("Frame", frame)
                frame = cv2.line(frame, (list1[0][0],list1[0][1]), (list1[1][0],list1[1][1]), (255,255,0), 2)
                frame = cv2.line(frame, (list1[1][0],list1[1][1]), (list1[2][0],list1[2][1]), (255,255,0), 2)
                frame = cv2.line(frame, (list1[2][0],list1[2][1]), (list1[3][0],list1[3][1]), (255,255,0), 2)
                # self.pub2.publish(list1)
                cv2.imshow("Frame", frame)
                # print(results)
                if cv2.waitKey(1) & 0xFF == ord('q'):0
                pass

                # msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                # self.pub.publish(msg)
            except CvBridgeError as e:
                print(e)



if __name__ == '__main__':
    parser = argparse.ArgumentParser('Camera Driver')
    parser.add_argument('source', nargs='?', type=str, default='2')
    parser.add_argument('-t', '--topic', type=str, default='/overhead_camera/image_raw')
    args = parser.parse_args()

    camera_driver = CameraDriver(args)
    try:
        camera_driver.publish()
    except ROSInterruptException as e:
        print(e)
