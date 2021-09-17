#!/usr/bin/env python3
import rospy
import cv2
import apriltag
import math
from sensor_msgs.msg import Image
from std_msgs.msg import Int32,String
from geometry_msgs.msg import Twist
#from cv_bridge import CvBridge,CvBridgeError
i,j,k,l=0,0,0,0
deg = 0.00
flag=0
reverse=0
turn=0
kp,ki,kd=1.00,0.01,0.01
list1=[[271,57],[271,214],[271, 57]]#coordinates of s1,turning point and d1
rev_turn=0
turn_fl1,turn_fl2,rev_fl,rev,stop,flag=0,0,0,0,0,0
class PubNode:
    def __init__(self):
        rospy.init_node("pub_guy")
        self.sub = rospy.Subscriber("apriltag_centre1",String,self.callback)
        self.pub1 = rospy.Publisher("wifi", String, queue_size = 10)
        self.rate = rospy.Rate(30)
        #self.msg = Twist()

    def wheel_speed(self,err,err_deg):
        base_speed = 90
        err_speed=kp*(err-err_deg)
        rs = int(base_speed-err_speed)
        ls = int(base_speed+err_speed)
        self.msg = str(rs)+','+str(ls)+','+'0'#right_motor speed+left_motor_speed+state of servo
        print("string = "+self.msg)
        try:
            self.pub1.publish(self.msg)
            # self.pub2.publish(1)
            self.rate.sleep()
        except rospy.ROSInterruptException as e:
            print(e)
    def bot_error(self,deg,x2,y2,x1,y1):
         global list1,turn,flag,turn_fl1,turn_fl2,i,j,stop
         l1 = str(x2)+" "+str(y2)
         a1=list(l1.split(" "))
         #print("angle"+str(deg))
         if int(a1[1])<list1[1][1]-30 and turn==0:
             print("in 1")
             err=list1[1][0]-int(a1[0])
             err_deg=90-math.degrees(math.acos((x2-x1)/math.sqrt((x2-x1)**2 + (y2-y1)**2)))
             self.wheel_speed(err,err_deg)
             j=j+1
         elif int(a1[1])>=list1[1][1]+30 and deg<178 and turn==0:
             print("in 2")
             rs = 0
             ls = -200
             self.msg = str(rs)+','+str(ls)+','+'0'
             turn=1
             #print(deg)
             try:
                 self.pub1.publish(self.msg)
                 self.rate.sleep()
             except rospy.ROSInterruptException as e:
                 print(e)
         elif int(a1[0])<=list1[2][0] and flag==0:
             print("in 4")
             rs=0
             ls=0
             self.msg = str(rs)+','+str(ls)+','+'1'
             #print("d1 stop")
             flag=1
             try:
                 self.pub1.publish(self.msg)
                 self.rate.sleep()
             except rospy.ROSInterruptException as e:
                 print(e)
         elif turn==1 and turn_fl1==0 and int(a1[1])>list1[1][1]-66 and deg>=178:
             turn_fl1=1
         elif turn==1 and int(a1[1])>list1[1][1]-66 and flag==0 and turn_fl1==1:
             print("in 3")
             err=list1[1][1]-int(a1[1])
             err_deg=90-math.degrees(math.acos((y2-y1)/math.sqrt((x2-x1)**2 + (y2-y1)**2)))
             self.wheel_speed(err,err_deg)
             i=i+1
             #print(err_deg)

    def rev_error(self,deg,x2,y2,x1,y1):
         global flag,list1,rev_turn,k,l,turn_fl2,rev_fl,rev
         l1 = str(x2)+" "+str(y2)
         a1=list(l1.split(" "))
         print("angle"+str(deg))
         if int(a1[0])<=list1[2][0]+20 and rev_turn==0 and rev==0:
             print("in 5")
             rs =-200
             ls = 0
             self.msg = str(rs)+','+str(ls)+','+'0'
             rev=1
             #print("Reverse")
             try:
                 self.pub1.publish(self.msg)
                 self.rate.sleep()
             except rospy.ROSInterruptException as e:
                 print(e)
         elif rev_turn==0 and rev_fl==0 and deg<=2 and rev==1:
             rev_fl=1
         elif int(a1[0])<=list1[1][0]-10 and rev_turn==0 and rev_fl==1 and rev==1:
             print("in 6")
             err=int(a1[1])-list1[1][1]
             err_deg=math.degrees(math.acos((y2-y1)/math.sqrt((x2-x1)**2 + (y2-y1)**2)))-90
             self.wheel_speed(err,err_deg)
             #print("rev_hori"+str(k))
             k = k+1
         elif int(a1[0])>list1[1][0]-10 and rev_turn==0:
             rs =-200
             ls = 0
             self.msg = str(rs)+','+str(ls)+','+'0'
             rev_turn=1
             print("in 7")
             try:
                 self.pub1.publish(self.msg)
                 self.rate.sleep()
             except rospy.ROSInterruptException as e:
                 print(e)
         elif int(a1[0])>list1[1][0]-66 and rev_turn==1 and turn_fl2==0 and deg>88:
             turn_fl2=1
         elif int(a1[0])>list1[1][0]-66 and rev_turn==1 and turn_fl2==1:
             print("in 8")
             err=-(list1[0][0]-int(a1[0]))
             err_deg=-(90-math.degrees(math.acos((x2-x1)/math.sqrt((x2-x1)**2 + (y2-y1)**2))))
             self.wheel_speed(err,err_deg)
             #print("Last"+str(l))
             l=l+1
         if int(a1[1])<list1[0][1]+10 and flag==1 and turn_fl2==1:
             rs = 0
             ls = 0
             self.msg = str(rs)+','+str(ls)+','+'0'
             #print("s1 stop")
             flag=2
             try:
                 self.pub1.publish(self.msg)
                 # self.pub2.publish(2)
                 self.rate.sleep()
                 exit()
             except rospy.ROSInterruptException as e:
                 print(e)

    def callback(self, data):
         global flag
         my_data = data.data
         print(my_data)
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
        rospy.spin()

    except rospy.ROSInterruptException as e:
        print(e)
