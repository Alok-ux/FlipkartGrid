#!/usr/bin/env python
import rospy
import cv2
import apriltag
from std_msgs.msg import Int32,String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge,CvBridgeError
import numpy as np
centre=()
ptG=()
d=[]
i=0
flag=0
pos=""
class listener():
    def __init__(self):
        rospy.init_node('listener',anonymous = True)
        self.sub = rospy.Subscriber("/overhead_camera/image_raw",Image,self.callback)
        self.pub = rospy.Publisher("april_pos",String,queue_size=10)
        self.rate = rospy.Rate(30)
        self.bridge = CvBridge()
        self.msg = Twist()
    def publish_pos(self):
            global pos
            while not rospy.is_shutdown():
                try: 
                    self.pub.publish(pos)
                    self.rate.sleep()
                    print(pos)
                except rospy.ROSInterruptException as e:
                    print(e)
    def callback(self,data):
        global ptG,centre,i,d,flag,pos
        try:
            self.image = self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.gray=cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(self.gray,(5,5),0)
            _,thresh = cv2.threshold(blur,65,255,cv2.THRESH_BINARY)
            _,contours,_ = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                   M = cv2.moments(cnt)
                   if cv2.contourArea(cnt)<100000 and cv2.contourArea(cnt)>100:
                       cx=int(M['m10']/M['m00'])
                       cy=int(M['m01']/M['m00'])
            options = apriltag.DetectorOptions(families="tag36h11")
            detector = apriltag.Detector()
            results = detector.detect(self.gray)
            #print(len(results))
            for r in results:
	            (ptA, ptB, ptC, ptD) = r.corners
	            ptB = (int(ptB[0]), int(ptB[1]))
	            ptC = (int(ptC[0]), int(ptC[1]))
	            ptD = (int(ptD[0]), int(ptD[1]))
	            ptA = (int(ptA[0]), int(ptA[1]))
	            centre = (int(r.center[0]), int(r.center[1]))
	            cv2.circle(self.image, centre, 3, (0, 0, 255), -1)  
                    #cv2.circle(self.image, ptA, 3, (0, 0, 255), -1)  
                    #cv2.circle(self.image, ptB, 3, (0, 0, 255), -1)  
                    cv2.circle(self.image, ptC, 3, (0, 0, 255), -1)  
                    cv2.circle(self.image, ptD, 3, (0, 0, 255), -1)
                    #cv2.putText(self.image,"A",ptA,cv2.FONT_HERSHEY_SIMPLEX,2,(255,0,0),1)  
                    #cv2.putText(self.image,"B",ptB,cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),1)
                    #cv2.putText(self.image,"C",ptC,cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),1)
                    #cv2.putText(self.image,"D",ptD,cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),1)
                    #ptE = (int((ptA[0]+ptB[0])/2), int((ptA[1]+ptB[1])/2))
                    #ptF = (int((ptB[0]+ptC[0])/2), int((ptB[1]+ptC[1])/2))
                    ptG = (int((ptC[0]+ptD[0])/2), int((ptC[1]+ptD[1])/2))
                    #ptH = (int((ptD[0]+ptA[0])/2), int((ptD[1]+ptA[1])/2))
                    #cv2.circle(self.image, ptE, 3, (0, 255, 255), -1)  
                    #cv2.circle(self.image, ptF, 3, (0, 255, 255), -1)  
                    #cv2.circle(self.image, ptG, 3, (0, 255, 255), -1)  
                    #cv2.circle(self.image, ptH, 3, (0, 255, 255), -1)
                    cv2.arrowedLine(self.image,centre,ptG,(255,0,0),3)
            i=i+1  
            #print(self.st1)
            cv2.imshow("rst",self.image)
            self.st1 = str(centre[0]) + " " + str(centre[1])
            self.st2 = str(ptG[0]) + " " + str(ptG[1])
            pos = self.st1 + " " + self.st2
            #print(self.st1)
                             
            
            if cv2.waitKey(20) == ord('q'):
                rospy.signal_shutdown("shutdown")
                cv2.destroyAllWindows()
           
            
        except CvBridgeError as e:
            print(e)
       
         #self.publish_
        #if abs(centre[1]-666)>5:
        #    print(abs(centre[1]-666))
        #    self.msg.linear.x = 0.05
        #    self.msg.angular.z = 0.
        #else:
        #    self.msg.linear.x = 0.0
        #    self.msg.angular.z = 0.0
           
        #self.pub.publish(self.msg)
        #self.rate.sleep()  
    
if __name__ == '__main__':

    obj = listener()
    try:
        #print(flag)
        obj.publish_pos()
        rospy.spin()
        print(flag)
        #print(ptG)
    except rospy.ROSInterruptException as f:
        print(f)

