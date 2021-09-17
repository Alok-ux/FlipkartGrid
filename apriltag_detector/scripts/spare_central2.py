#!/usr/bin/env python
import rospy
import cv2
import apriltag
import math
#from sensor_msgs.msg import Image
from std_msgs.msg import Int32,String
s1 = (0,1) #coordinates of s1,correct value will be enterd
m1 = (2,3) #coordinates of d1,correct value will bee entered
class listener():
    def __init__(self):
        rospy.init_node('listener',anonymous = True)
        self.pub1 = rospy.Publisher("apriltag_centre1",String,queue_size = 10)
        self.rate = rospy.Rate(30)
    def april(self):
        cap = cv2.VideoCapture(2)
        cap.set(3,1280)
        cap.set(4,1024)

        if not cap.isOpened():
            print("Camera cant be opened")
            exit()

        while True:
            ret, frame = cap.read()
            if not ret:
                continue

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
                cv2.rectangle(frame,pta,ptc,(0,255,0),2)
                cv2.circle(frame,cn,3,(0,0,255),-1)
                cv2.circle(frame,pte,3,(0,0,255),-1)
                cv2.line(frame,s1,m1,(0,0,255),2)
                cv2.line(frame,cn,m1,(0,255,0),2)
                dot_pro = ((s1[0]-m1[0])*(cn[0]-m1[0]))+((s1[1]-m1[1])*(cn[1]-m1[1]))
                mod_a= math.sqrt((s1[0]-m1[0])**2+(s1[1]-m1[1])**2)
                mod_b = math.sqrt((cn[0]-m1[0])**2+(s1[1]-m1[1])**2)
                deg = math.degrees(math.acos((dot_pro)/(mod_a*mod_b)))
                cv2.putText(frame, str(deg), (m1[0]+10,m1[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA)
                self.str = "("+str(cn[0])+","+str(cn[1])+")"+"("+str(pte[0])+","+"("+str(pte[1])+")"
                
            cv2.imshow("Frame", frame)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
        self.pub1(self.str)
        cv2.destroyAllWindows()
if __name__ == '__main__':
    obj = listener()
    try:
        obj.april()
    except Exception as e:
        print(e)