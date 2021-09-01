#!/usr/bin/env python
import rospy
import cv2
import apriltag
import math
from sensor_msgs.msg import Image
from std_msgs.msg import Int32,String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge,CvBridgeError
i,j,k,l=0,0,0,0
deg = 0.00
flag=0
reverse=0
turn=0
kp,ki,kd=0.1,0.0,0.02#kp=0.01
kp1,kd1,p_err1=0.03,0.01,0
list1=[[542,87],[542,602],[85,602]]
rev_turn=0
p_err=0
turn_fl1,turn_fl2,rev_fl,rev,stop,flag=0,0,0,0,0,0
class PubNode:
    def __init__(self):

        rospy.init_node("pub_guy")
        self.sub = rospy.Subscriber("april_pos",String,self.callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.rate = rospy.Rate(40)
        self.msg = Twist()
    def stop(self):
        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.z = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.x = 0.0
             #print("d1 stop")
             #flag=1
        try:
            self.pub.publish(self.msg)
            self.rate.sleep()
        except rospy.ROSInterruptException as e:
            print(e)
    def wheel_speed(self,err,ang_err):
        global kp,kd,p_err
        base_speed = 0.8
        #D = err-p_err
        err_speed=kp*(err-ang_err)
        #p_err = err
        self.msg.linear.x = base_speed    #0.1
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.z = err_speed
        self.msg.angular.y = 0.0
        self.msg.angular.x = 0.0
        print("value = "+str(err_speed))
        try:
            self.pub.publish(self.msg)
            self.rate.sleep()
        except rospy.ROSInterruptException as e:
            print(e)
    def wheel_speed_hr(self,err,ang_err):
        global kp1,kd1,p_err
        base_speed = 0.8
        #D = err-p_err
        err_speed=kp1*(err-ang_err)
        #p_err1 = err
        self.msg.linear.x = base_speed    #0.1
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.z = err_speed
        self.msg.angular.y = 0.0
        self.msg.angular.x = 0.0
        print("value = "+str(err_speed))
        #print(str(rev_turn)+" "+str(rev))
        try:
            self.pub.publish(self.msg)
            self.rate.sleep()
        except rospy.ROSInterruptException as e:
            print(e)  
    def bot_error(self,deg,x2,y2,x1,y1):
         global list1,turn,flag,turn_fl1,turn_fl2,i,j,stop
         l1 = str(x2)+" "+str(y2)
         a1=list(l1.split(" "))
         #print("angle"+str(deg))
         if int(a1[1])<331 and turn==0: #initial limit=330
             #print("vertical"+str(j))
             err=list1[1][0]-int(a1[0])
             err_deg=90-math.degrees(math.acos((x2-x1)/math.sqrt((x2-x1)**2 + (y2-y1)**2)))
             self.wheel_speed(err,err_deg)
             j=j+1
         elif int(a1[1])>=331 and deg<178 and turn==0:#changed #initial limit=330
             print(a1[1])
             #self.stop()
             self.msg.linear.x = 0.0
             self.msg.linear.y = 0.0
             self.msg.linear.z = 0.0
             self.msg.angular.z = -0.05
             self.msg.angular.y = 0.0
             self.msg.angular.x = 0.0
             turn=1
             #print(turn)
             try:
                 self.pub.publish(self.msg)
                 self.rate.sleep()
             except rospy.ROSInterruptException as e:
                 print(e)
         elif int(a1[0])<=85+264 and flag==0:
             self.msg.linear.x = 0.0
             self.msg.linear.y = 0.0
             self.msg.linear.z = 0.0
             self.msg.angular.z = 0.0
             self.msg.angular.y = 0.0
             self.msg.angular.x = 0.0
             #print("d1 stop")
             flag=1
             try:
                 self.pub.publish(self.msg)
                 self.rate.sleep()
             except rospy.ROSInterruptException as e:
                 print(e)
         elif turn==1 and turn_fl1==0 and int(a1[1])>list1[1][1]-66 and deg>=178: #changed
             turn_fl1=1 
             print(a1[1])       
         elif turn==1 and int(a1[1])>list1[1][1]-66 and flag==0 and turn_fl1==1:   #chamged
             #print("Hori"+str(i))
             err=602-int(a1[1])
             err_deg=90-math.degrees(math.acos((y2-y1)/math.sqrt((x2-x1)**2 + (y2-y1)**2)))
             self.wheel_speed_hr(err,err_deg)
             i=i+1
             #print(err_deg)
        
    def rev_error(self,deg,x2,y2,x1,y1):
         global flag,list1,rev_turn,k,l,turn_fl2,rev_fl,rev
         l1 = str(x2)+" "+str(y2)
         a1=list(l1.split(" "))
         #print(str(a1[0])+" "+str(rev_turn)+" "+str(rev_fl)+" "+str(rev))
         if int(a1[0])<=list1[2][0]+264 and rev_turn==0 and rev==0:#rev_turn==0
             self.msg.linear.x = 0.0
             self.msg.linear.y = 0.0
             self.msg.linear.z = 0.0
             self.msg.angular.z = 0.05
             self.msg.angular.y = 0.0
             self.msg.angular.x = 0.0
             rev=1
             #print("Reverse")
             try:
                 self.pub.publish(self.msg)
                 self.rate.sleep()
             except rospy.ROSInterruptException as e:
                 print(e)
         elif rev_turn==0 and rev_fl==0 and deg<=2 and rev==1:#rev_turn replaced
             rev_fl=1    
         elif int(a1[0])<list1[1][0]-60 and rev_turn==0 and rev_fl==1 and rev==1:#rev_turn replaced
             err=int(a1[1])-list1[1][1]
             err_deg=math.degrees(math.acos((y2-y1)/math.sqrt((x2-x1)**2 + (y2-y1)**2)))-90
             self.wheel_speed(err,err_deg)
             print("rev_hori"+str(k))
             k = k+1
         elif int(a1[0])>=list1[1][0]-250 and rev_turn==0:#changed#rev_turn replaced
             print(a1[0])
             #self.stop()
             self.msg.linear.x = 0.0
             self.msg.linear.y = 0.0
             self.msg.linear.z = 0.0
             self.msg.angular.z = 0.05
             self.msg.angular.y = 0.0
             self.msg.angular.x = 0.0
             rev_turn=1
             
             try:
                 self.pub.publish(self.msg)
                 self.rate.sleep()
             except rospy.ROSInterruptException as e:
                 print(e)
         elif int(a1[0])>list1[1][0]-66 and rev_turn==0 and turn_fl2==0 and deg>88:#changed  #rev_turn replaced
             turn_fl2=1
         elif int(a1[0])>list1[1][0]-66 and rev_turn==0 and turn_fl2==1:#changed   #rev_turn replaced
             err=-(list1[0][0]-int(a1[0]))
             err_deg=-(90-math.degrees(math.acos((x2-x1)/math.sqrt((x2-x1)**2 + (y2-y1)**2))))
             self.wheel_speed(err,err_deg)
             #print("Last"+str(l))
             l=l+1
         if int(a1[1])<list1[0][1]+10 and flag==1 and turn_fl2==1:
             self.msg.linear.x = 0.0
             self.msg.linear.y = 0.0
             self.msg.linear.z = 0.0
             self.msg.angular.z = 0.0
             self.msg.angular.y = 0.0
             self.msg.angular.x = 0.0
             #print("s1 stop")
             flag=2
             try:
                 self.pub.publish(self.msg)
                 self.rate.sleep()
             except rospy.ROSInterruptException as e:
                 print(e)               
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
