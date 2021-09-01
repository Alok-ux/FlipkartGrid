#!/usr/bin/env python
import rospy
import cv2
import apriltag
import math
from sensor_msgs.msg import Image
from std_msgs.msg import Int32,String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge,CvBridgeError
cy=0
i=0
stop=0
angle = 0.00
my=0
flag=0
reverse=0
turn=0
kp=0.01
ki=0.00
kd=0.00
I=0
previous_er=0
error1=180
error=0
error2=172
flip=0
error3=2
le=0
class PubNode:
    def __init__(self):

        rospy.init_node("pub_guy")
        self.sub = rospy.Subscriber("april_pos",String,self.callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.rate = rospy.Rate(10)
        self.msg = Twist()
        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.z = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.x = 0.0
    def pid(self,er):
        global kp,ki,kd,I,previous_er
        p = abs(er)
        I = I+abs(er)
        d = abs(er) - previous_er
        val = kp*p + ki*I +kd*d
        previous_er = abs(er)
        return val

    def motion(self,s,e):
        if e<0:
            self.msg.linear.x=0.05
            self.msg.angular.z=s
        elif e>0:
            self.msg.linear.x=0.05
            self.msg.angular.z=-s
        elif e == 0:
            self.msg.linear.x=0.2
            self.msg.angular.z=0.0
    def publish_method(self):
        global cy,i,stop,angle,my,flag
        while not rospy.is_shutdown():
            '''if 666-cy>13:
                self.msg.linear.x = 0.1
                self.msg.linear.y = 0.0
                self.msg.linear.z = 0.0
                self.msg.angular.z = 0.0
                self.msg.angular.y = 0.0
                self.msg.angular.x = 0.0
            elif 666-cy<=13:
                if angle<173.00 and flag==0:
                    self.msg.linear.x = 0.0
                    self.msg.linear.y = 0.0
                    self.msg.linear.z = 0.0
                    self.msg.angular.z = -0.1
                    self.msg.angular.y = 0.0
                    self.msg.angular.x = 0.0
                elif angle>=173.00 or flag==1:
                    flag=1
                    if my<cy:
                        self.msg.linear.x = 0.0
                        self.msg.linear.y = 0.0
                        self.msg.linear.z = 0.0
                        self.msg.angular.z = -0.1
                        self.msg.angular.y = 0.0
                        self.msg.angular.x = 0.0
                    elif my>cy:
                        self.msg.linear.x = 0.
                        self.msg.linear.y = 0.0
                        self.msg.linear.z = 0.0
                        self.msg.angular.z = 0.1
                        self.msg.angular.y = 0.0
                        self.msg.angular.x = 0.0
                    elif my==cy:
                        self.msg.linear.x = 0.1
                        self.msg.linear.y = 0.0
                        self.msg.linear.z = 0.0
                        self.msg.angular.z = 0.0
                        self.msg.angular.y = 0.0
                        self.msg.angular.x = 0.0   
                #else:
                #    pass
            i = i+1
            #print(666-li_y>5)'''
            self.pub.publish(self.msg)
            self.rate.sleep()

    
    def callback(self, data):
         global cy,angle,kp,ki,kd,reverse,turn,error1,error,error2,flip,error3
         my_data = data.data
         li = list(my_data.split(" "))
         cx = int(li[0])
         cy = int(li[1])
         mx = int(li[2])
         my = int(li[3])
         if mx-cx==0:
            angle = 90.00
         else:
            angle = (180/3.14)*math.acos((cx-mx)/math.sqrt((mx-cx)**2 + (my-cy)**2))
         #print(angle)
         if 666-cy>10 and reverse==0:
             error = 90.00-angle
             value = self.pid(error)
             self.motion(value,error)
             #if error==0:          
             #print(error)
         elif 666-cy<=10 and reverse==0:
             if error1>1:
                 error1 = 180.00 - angle
                 #p = abs(error1)
                 '''I = I+abs(error1)
                 d = abs(er) - previous_er'''
                 #val1 = kp*p     
                 self.msg.linear.x=0.0
                 self.msg.angular.z=-0.1
             elif error1<=1:
                 turn=1
         if turn==1 and cx>95 and flip==0:#changed from this
              if my>cy:
                  error=2*(angle-180)
              elif my<cy:
                  error=2*(180-angle)
              value = self.pid(error)
              self.motion(value,error)
         elif turn==1 and cx<=95 and flip==0:
                 self.msg.linear.x=0.0
                 self.msg.angular.z=0.0
                 flip=1
         if flip==1 and turn==1 and reverse==0:
                 if error2<173:
                     error2 = 180.00-angle
                     self.msg.linear.x=0.0
                     self.msg.angular.z=0.1
                 elif error2>=173:
                     reverse=1
                     self.msg.linear.x=0.0
                     self.msg.angular.z=0.0
         elif reverse==1 and turn==1 and flip==1 and cx<594:
              if my>cy:
                  error=2*(angle-0)
              elif my<cy:
                  error=2*(0-angle)
              value = self.pid(error)
              self.motion(value,error)
         if reverse==1 and turn==1 and flip==1 and cx>=594:
             if error3>1:
                 error3 = 90.00 - angle
                 #p = abs(error1)
                 '''I = I+abs(error1)
                 d = abs(er) - previous_er'''
                 #val1 = kp*p     
                 self.msg.linear.x=0.0
                 self.msg.angular.z=0.1
             elif error3<=1:
                 turn=2
         if turn==2 and cy>=100:
             error=2*(angle-90.00)
             value = self.pid(error)
             self.motion(value,error)
         elif cy<100 and turn==2:
             stop=0
             self.msg.linear.x=0.0
             self.msg.angular.z=0.0                        
                 
                    
         print(angle)    


if __name__ == '__main__':
    obj = PubNode()
    try:
        obj.publish_method()
        rospy.spin()
    except Exception as e:
        print(e)  
