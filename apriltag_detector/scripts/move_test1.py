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
deg = 0.00
my=0
flag=0
reverse=0
turn=0
kp,ki,kd=0.01,0.01,0.01
I=0
previous_er=0
error1=180
error=0
error2=172
flip=0
error3=2
le=0
list1=[[542,87],[542,602],[85,602]]
rev_turn=0
class PubNode:
    def __init__(self):

        rospy.init_node("pub_guy")
        self.sub = rospy.Subscriber("april_pos",String,self.callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.rate = rospy.Rate(30)
        self.msg = Twist()
        '''self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.z = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.x = 0.0
        try:
            self.pub.publish(self.msg)
            self.rate.sleep()
        except rospy.ROSInterruptException as e:
            print(e)''' 
    def wheel_speed(self,err,ang_err):
        base_speed = 0.1
        err_speed=kp*(err-ang_err)
        self.msg.linear.x = 0.1
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.z = err_speed
        self.msg.angular.y = 0.0
        self.msg.angular.x = 0.0
        print("ENtered")
        try:
            self.pub.publish(self.msg)
            self.rate.sleep()
        except rospy.ROSInterruptException as e:
            print(e) 
    def bot_error(self,deg,x2,y2,x1,y1):
         global list1,turn,flag
         l1 = str(x2)+" "+str(y2)
         a1=list(l1.split(" "))
         if int(a1[1])<list1[1][1]-10 and turn==0:
             err=list1[1][0]-int(a1[0])
             err_deg=90-math.degrees(math.acos((x2-x1)/math.sqrt((x2-x1)**2 + (y2-y1)**2)))
             self.wheel_speed(err,err_deg)
         elif int(a1[1])>=list1[1][1]-10 and deg<=170 and turn==0:
             self.msg.linear.x = 0.0
             self.msg.linear.y = 0.0
             self.msg.linear.z = 0.0
             self.msg.angular.z = -0.1
             self.msg.angular.y = 0.0
             self.msg.angular.x = 0.0
             turn=1
             #print(deg)
             try:
                 self.pub.publish(self.msg)
                 self.rate.sleep()
             except rospy.ROSInterruptException as e:
                 print(e)
         elif int(a1[0])<=85 and flag==0:
             self.msg.linear.x = 0.0
             self.msg.linear.y = 0.0
             self.msg.linear.z = 0.0
             self.msg.angular.z = 0.0
             self.msg.angular.y = 0.0
             self.msg.angular.x = 0.0
             print("d1 stop")
             flag=1
             try:
                 self.pub.publish(self.msg)
                 self.rate.sleep()
             except rospy.ROSInterruptException as e:
                 print(e)
         elif turn==1 and int(a1[1])>list1[1][1]-66 and deg>170 and flag==0:
             err=list1[1][1]-int(a1[1])
             err_deg=90-math.degrees(math.acos((y2-y1)/math.sqrt((x2-x1)**2 + (y2-y1)**2)))
             self.wheel_speed(err,err_deg)
             #print(err_deg)
        
    def rev_error(self,deg,x2,y2,x1,y1):
         global flag,list1,rev_turn
         l1 = str(x2)+" "+str(y2)
         a1=list(l1.split(" "))
         if int(a1[0])<=list1[2][0]+10 and rev_turn==0 and deg>=5:
             self.msg.linear.x = 0.0
             self.msg.linear.y = 0.0
             self.msg.linear.z = 0.0
             self.msg.angular.z = 0.1
             self.msg.angular.y = 0.0
             self.msg.angular.x = 0.0
             try:
                 self.pub.publish(self.msg)
                 self.rate.sleep()
             except rospy.ROSInterruptException as e:
                 print(e)    
         elif int(a1[0])<=list1[1][0]-10 and rev_turn==0:
             err=int(a1[1])-list1[1][1]
             err_deg=math.degrees(math.acos((y2-y1)/math.sqrt((x2-x1)**2 + (y2-y1)**2)))-90
             self.wheel_speed(err,err_deg)
         elif int(a1[0])>=list1[1][0]-10 and deg<=85 and rev_turn==0:
             self.msg.linear.x = 0.0
             self.msg.linear.y = 0.0
             self.msg.linear.z = 0.0
             self.msg.angular.z = 0.1
             self.msg.angular.y = 0.0
             self.msg.angular.x = 0.0
             rev_turn=1
             #print(deg)
             try:
                 self.pub.publish(self.msg)
                 self.rate.sleep()
             except rospy.ROSInterruptException as e:
                 print(e)
         elif int(a1[0])>list1[1][0]-66 and rev_turn==1 and deg>85:
             err=-(list1[0][0]-int(a1[0]))
             err_deg=-(90-math.degrees(math.acos((x2-x1)/math.sqrt((x2-x1)**2 + (y2-y1)**2))))
             print("Last")
             self.wheel_speed(err,err_deg)
         '''elif int(a1[1])<list1[0][1]+10 and flag==1:
             self.msg.linear.x = 0.0
             self.msg.linear.y = 0.0
             self.msg.linear.z = 0.0
             self.msg.angular.z = 0.0
             self.msg.angular.y = 0.0
             self.msg.angular.x = 0.0
             print("s1 stop")
             flag=2
             try:
                 self.pub.publish(self.msg)
                 self.rate.sleep()
             except rospy.ROSInterruptException as e:
                 print(e)'''       
         '''if turn==1 and int(a1[0])<=list1[2][0]+9 and int(a1[1])>=list1[1][1]-66 and deg>=10:#from here
             self.msg.linear.x = 0.0
             self.msg.linear.y = 0.0
             self.msg.linear.z = 0.0
             self.msg.angular.z = 0.1
             self.msg.angular.y = 0.0
             self.msg.angular.x = 0.0
             turn=2
             print("entering d1")
             #print(deg)
             try:
                 self.pub.publish(self.msg)
                 self.rate.sleep()
             except rospy.ROSInterruptException as e:
                 print(e)
         elif turn==2 and deg<10 and int(a1[0])<=list1[1][0]+9 and int(a1[1])>=list1[1][1]-66:
             err=list1[1][1]-int(a1[1])
             err_deg=math.degrees(math.acos((y1-y2)/math.sqrt((x2-x1)**2 + (y2-y1)**2)))-90
             self.wheel_speed(err,err_deg)
             print("reverse")'''         
         #print(turn) 
 
    def callback(self, data):
         global flag
         my_data = data.data
         li = list(my_data.split(" "))
         cx = int(li[0])
         cy = int(li[1])
         mx = int(li[2])
         my = int(li[3])
         deg = math.degrees(math.acos((cx-mx)/math.sqrt((mx-cx)**2 + (my-cy)**2)))
         if flag==0:
             self.bot_error(deg,cx,cy,mx,my)
         elif flag==1:
             self.rev_error(deg,cx,cy,mx,my)
         #print(angle)
    


if __name__ == '__main__':
    obj = PubNode()
    try:
        #obj.publish_method()
        rospy.spin()
       
    except rospy.ROSInterruptException as e:
        print(e)
