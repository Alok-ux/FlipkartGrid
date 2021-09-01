#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import apriltag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
flag=0
cx=cy=0
d=[]
i=0
class listener():
    def __init__(self):
        rospy.init_node('listener',anonymous = True)
        self.sub = rospy.Subscriber("/overhead_camera/image_raw",Image,self.callback)
        #self.rate = rospy.Rate(1)
        self.bridge = CvBridge()

    def callback(self,data):
        global flag,cx,cy,i
        try:
            self.image = self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.hsv=cv2.cvtColor(self.image,cv2.COLOR_BGR2HSV)
            #for d1
            low_d1=np.array([74,2,2])
            high_d1=np.array([94,255,255])
            d1_mask=cv2.inRange(self.hsv,low_d1,high_d1)
            d1=cv2.bitwise_and(self.image,self.image,mask=d1_mask)
            self.gray=cv2.cvtColor(d1,cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(self.gray,(5,5),0)
            _,thresh = cv2.threshold(blur,65,255,cv2.THRESH_BINARY)
            _,contours,_ = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt)<100000 and cv2.contourArea(cnt)>3000:
                    #print(cv2.contourArea(cnt))
                    M = cv2.moments(cnt)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cv2.circle(d1,(cx,cy),3,(255,0,0),-1)
                    if cy>100 and i==0:
                        d.append([cx,cy])
            #for d2
            low_d2=np.array([5,20,62])
            high_d2=np.array([20,255,255])
            d2_mask=cv2.inRange(self.hsv,low_d2,high_d2)
            d2=cv2.bitwise_and(self.image,self.image,mask=d2_mask)
            self.gray=cv2.cvtColor(d2,cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(self.gray,(5,5),0)
            _,thresh = cv2.threshold(blur,65,255,cv2.THRESH_BINARY)
            _,contours,_ = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt)<100000 and cv2.contourArea(cnt)>3000:
                    print(cv2.contourArea(cnt))
                    M = cv2.moments(cnt)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cv2.circle(d2,(cx,cy),3,(255,0,0),-1)
                    if cy>100 and i==0:
                        d.append([cx,cy])
             #for d3
            low_d3=np.array([20,20,62])
            high_d3=np.array([40,255,255])
            d3_mask=cv2.inRange(self.hsv,low_d3,high_d3)
            d3=cv2.bitwise_and(self.image,self.image,mask=d3_mask)
            self.gray=cv2.cvtColor(d3,cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(self.gray,(5,5),0)
            _,thresh = cv2.threshold(blur,65,255,cv2.THRESH_BINARY)
            _,contours,_ = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt)<100000 and cv2.contourArea(cnt)>3500:
                    #print(cv2.contourArea(cnt))
                    M = cv2.moments(cnt)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cv2.circle(d3,(cx,cy),3,(255,0,0),-1)
                    if cy>100 and i==0:
                        d.append([cx,cy])
            #for d4
            low_d4=np.array([104,20,62])
            high_d4=np.array([130,255,255])
            d4_mask=cv2.inRange(self.hsv,low_d4,high_d4)
            d4=cv2.bitwise_and(self.image,self.image,mask=d4_mask)
            self.gray=cv2.cvtColor(d4,cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(self.gray,(5,5),0)
            _,thresh = cv2.threshold(blur,65,255,cv2.THRESH_BINARY)
            _,contours,_ = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt)<100000 and cv2.contourArea(cnt)>3000:
                    #print(cv2.contourArea(cnt))
                    M = cv2.moments(cnt)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cv2.circle(d4,(cx,cy),3,(255,0,0),-1)
                    if cy>100 and i==0:
                        d.append([cx,cy])    
            cv2.imshow("rst",d4)
            i=i+1
            if cv2.waitKey(20) == ord('q'):
                rospy.signal_shutdown("shutdown")
                cv2.destroyAllWindows()
        except CvBridgeError as e:
            print(e)
if __name__ == '__main__':
    obj = listener()
    try:
        rospy.spin()
        print(d)
    except rospy.ROSInterruptException as f:
        print(f)
