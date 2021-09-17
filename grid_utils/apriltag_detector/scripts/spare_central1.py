#!/usr/bin/env python3
import rospy
import cv2
import apriltag
from sensor_msgs.msg import Image
from std_msgs.msg import Int32,String
from cv_bridge import CvBridge, CvBridgeError

list= [[274,56],[270,315],[]]

class listener():
    def __init__(self):
        rospy.init_node('listener',anonymous = True)
        self.pub1 = rospy.Publisher("apriltag_centre1",String,queue_size = 10)
        self.sub1 = rospy.Subscriber("/overhead_camera/image_raw",Image,self.april)
        self.rate = rospy.Rate(30)
        self.bridge = CvBridge()
    def april(self, data):
        # cap = cv2.VideoCapture(2)
        # cap.set(3,1280)
        # cap.set(4,1024)
        #
        # if not cap.isOpened():
        #     print("Camera cant be opened")
        #     exit()
        #
        # while True:
        #     ret, frame = cap.read()
        #     if not ret:
        #         continue
        frame=self.bridge.imgmsg_to_cv2(data, 'bgr8')
        #frame = frame[400: -400, 400: -400]
        #frame = cv2.resize(frame, (0, 0), fx=3, fy=3)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #ret, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        #contours = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        #for cnt in contours:
        #    M = cv2.moments(cnt)
        #    if M["m00"]!=0:
        #        cX = int(M["m10"] / M["m00"])
        #        cY = int(M["m01"] / M["m00"])
        #        cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
        #        cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
        options = apriltag.DetectorOptions('tag36h11')
        detector = apriltag.Detector(options)
        results = detector.detect(gray)
        for i in results:
            pta , ptb ,ptc ,ptd = i.corners
            cn = i.center
            pta = (int(pta[0]),int(pta[1]))
            ptb = (int(ptb[0]),int(ptb[1]))
            ptc = (int(ptc[0]),int(ptc[1]))
            ptd = (int(ptd[0]),int(ptd[1]))
            pte = (int((pta[0]+ptb[0])/2),int((pta[1]+ptb[1])/2))
            cn = (int(cn[0]),int(cn[1]))
            print("center: ",cn[0]," ",cn[1])
            cv2.rectangle(frame,pta,ptc,(0,255,0),2)
            cv2.circle(frame,cn,3,(0,0,255),-1)
            cv2.circle(frame,pte,3,(0,0,255),-1)
            self.str = str(cn[0])+" "+str(cn[1])+" "+str(pte[0])+" "+str(pte[1])
        frame = cv2.line(frame, (list[1][0],list[1][1]), (list[0][0],list[0][1]), (0,255,0), 2)
        cv2.imshow("Frame", frame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            pass
        self.pub1.publish(self.str)

if __name__ == '__main__':
    obj = listener()
    try:
        rospy.spin()
    except Exception as e:
        print(e)
