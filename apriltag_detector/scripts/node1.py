#!/usr/bin/env python
import rospy
import cv2
import apriltag
from sensor_msgs.msg import Image
from std_msgs.msg import Int32,String
from cv_bridge import CvBridge,CvBridgeError

v = 0


class listener():
    def __init__(self):
        rospy.init_node('listener',anonymous = True)
        self.sub = rospy.Subscriber("/overhead_camera/image_raw",Image,self.callback)
        self.sub1 = rospy.Subscriber("bot_a",Int32,self.callback1)
        self.sub2 = rospy.Subscriber("bot_b",Int32,self.callback2)
        self.sub3 = rospy.Subscriber("bot_c",Int32,self.callback3)
        self.sub4 = rospy.Subscriber("bot_d",Int32,self.callback4)
        self.pub1 = rospy.Publisher("apriltag_centre1",String,queue_size = 10)
        self.pub2 = rospy.Publisher("apriltag_centre2",String,queue_size = 10)
        self.pub3 = rospy.Publisher("apriltag_centre3",String,queue_size = 10)
        self.pub4 = rospy.Publisher("apriltag_centre4",String,queue_size = 10)
        self.rate = rospy.Rate(30)
        self.bridge = CvBridge()

    def callback(self,data):
        global v
        if v == 0:
            try :
                self.image = self.bridge.imgmsg_to_cv2(data,"bgr8")
                self.gray = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)

                options = apriltag.DetectorOptions('tag36h11')
                detector = apriltag.Detector(options)
                results = detector.detect(self.gray)

                for i in results:
                    pta , ptb ,ptc ,ptd = i.corners
                    cn = i.center
                    pta = (int(pta[0]),int(pta[1]))
                    ptb = (int(ptb[0]),int(ptb[1]))
                    ptc = (int(ptc[0]),int(ptc[1]))
                    ptd = (int(ptd[0]),int(ptd[1]))
                    pte = (int((pta[0]+ptb[0])/2),int((pta[1]+ptb[1])/2))
                    cn = (int(cn[0]),int(cn[1]))
                    self.str1 = "("+str(cn[0])+","+str(cn[1])+")"
                    self.str2 = "("+str(pte[0])+","+str(pte[1])+")"
                    self.stri = self.str1+self.str2
                    self.publ1(self.stri)

                    cv2.rectangle(self.image,pta,ptc,(0,255,0),2)
                    cv2.circle(self.image,cn,3,(0,0,255),-1)
                    cv2.circle(self.image,pte,3,(0,0,255),-1)


            except CvBridgeError as e:
                print(e)

        elif v == 1:
            try :
                self.image = self.bridge.imgmsg_to_cv2(data,"bgr8")
                self.gray = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)

                options = apriltag.DetectorOptions('tag36h11')
                detector = apriltag.Detector(options)
                results = detector.detect(self.gray)

                for i in results:
                    pta , ptb ,ptc ,ptd = i.corners
                    cn = i.center
                    pta = (int(pta[0]),int(pta[1]))
                    ptb = (int(ptb[0]),int(ptb[1]))
                    ptc = (int(ptc[0]),int(ptc[1]))
                    ptd = (int(ptd[0]),int(ptd[1]))
                    pte = (int((pta[0]+ptb[0])/2),int((pta[1]+ptb[1])/2))
                    cn = (int(cn[0]),int(cn[1]))
                    self.str1 = "("+str(cn[0])+","+str(cn[1])+")"
                    self.str2 = "("+str(pte[0])+","+str(pte[1])+")"
                    self.stri = self.str1+self.str2
                    self.publ2(self.stri)

                    cv2.rectangle(self.image,pta,ptc,(0,255,0),2)
                    cv2.circle(self.image,cn,3,(0,0,255),-1)
                    cv2.circle(self.image,pte,3,(0,0,255),-1)


            except CvBridgeError as e:
                print(e)
        elif v == 2:
            try :
                self.image = self.bridge.imgmsg_to_cv2(data,"bgr8")
                self.gray = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)

                options = apriltag.DetectorOptions('tag36h11')
                detector = apriltag.Detector(options)
                results = detector.detect(self.gray)

                for i in results:
                    pta , ptb ,ptc ,ptd = i.corners
                    cn = i.center
                    pta = (int(pta[0]),int(pta[1]))
                    ptb = (int(ptb[0]),int(ptb[1]))
                    ptc = (int(ptc[0]),int(ptc[1]))
                    ptd = (int(ptd[0]),int(ptd[1]))
                    pte = (int((pta[0]+ptb[0])/2),int((pta[1]+ptb[1])/2))
                    cn = (int(cn[0]),int(cn[1]))
                    self.str1 = "("+str(cn[0])+","+str(cn[1])+")"
                    self.str2 = "("+str(pte[0])+","+str(pte[1])+")"
                    self.stri = self.str1+self.str2
                    self.publ3(self.stri)

                    cv2.rectangle(self.image,pta,ptc,(0,255,0),2)
                    cv2.circle(self.image,cn,3,(0,0,255),-1)
                    cv2.circle(self.image,pte,3,(0,0,255),-1)


            except CvBridgeError as e:
                print(e)
        elif v == 3:
            try :
                self.image = self.bridge.imgmsg_to_cv2(data,"bgr8")
                self.gray = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)

                options = apriltag.DetectorOptions('tag36h11')
                detector = apriltag.Detector(options)
                results = detector.detect(self.gray)

                for i in results:
                    pta , ptb ,ptc ,ptd = i.corners
                    cn = i.center
                    pta = (int(pta[0]),int(pta[1]))
                    ptb = (int(ptb[0]),int(ptb[1]))
                    ptc = (int(ptc[0]),int(ptc[1]))
                    ptd = (int(ptd[0]),int(ptd[1]))
                    pte = (int((pta[0]+ptb[0])/2),int((pta[1]+ptb[1])/2))
                    cn = (int(cn[0]),int(cn[1]))
                    self.str1 = "("+str(cn[0])+","+str(cn[1])+")"
                    self.str2 = "("+str(pte[0])+","+str(pte[1])+")"
                    self.stri = self.str1+self.str2
                    self.publ4(self.stri)

                    cv2.rectangle(self.image,pta,ptc,(0,255,0),2)
                    cv2.circle(self.image,cn,3,(0,0,255),-1)
                    cv2.circle(self.image,pte,3,(0,0,255),-1)


            except CvBridgeError as e:
                print(e)


    def callback1(self,data):
        global v
        v = data
        print(v)

    def callback2(self,data):
        global v
        v = data
        print(v)

    def callback3(self,data):
        global v
        v = data
        print(v)

    def callback4(self,data):
        global v
        v = data
        print(v)


    def publ1(self,arg):
        while not rospy.is_shutdown():
            try:
                self.pub1.publish(arg)
                self.rate.sleep()
                break
            except rospy.ROSInterruptException as e:
                print(e)
                break

    def publ2(self,arg):
        while not rospy.is_shutdown():
            try:
                self.pub2.publish(arg)
                self.rate.sleep()
                break
            except rospy.ROSInterruptException as e:
                print(e)
                break

    def publ3(self,arg):
        while not rospy.is_shutdown():
            try:
                self.pub3.publish(arg)
                self.rate.sleep()
                break
            except rospy.ROSInterruptException as e:
                print(e)
                break

    def publ4(self,arg):
        while not rospy.is_shutdown():
            try:
                self.pub4.publish(arg)
                self.rate.sleep()
                break
            except rospy.ROSInterruptException as e:
                print(e)
                break


if __name__ == '__main__':
    obj = listener()
    try:
        rospy.spin()
        print("entering2")
    except Exception as e:
        print(e)
