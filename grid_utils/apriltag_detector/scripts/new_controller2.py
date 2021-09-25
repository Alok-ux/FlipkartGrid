#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
import math
from dynamic_reconfigure.server import Server
from apriltag_detector.cfg import dynamic_reconfigureConfig

kp = 20
# list1= [[304,112],[278,361],[278,361],]
list1= [[317,91],[297,347],[317,91]]
deg = 0
w1=1
w2=1

class listen():
    def __init__(self):
        rospy.init_node("controller",anonymous=True)
        self.sub = rospy.Subscriber("apriltag_centre1",String,self.callback)
        self.ack = rospy.Subscriber("recv",String,self.turnComplete)
        self.pub1 = rospy.Publisher("wifi",String,queue_size=1)
        self.pub = rospy.Publisher("err",String,queue_size=1)
        srv = Server(dynamic_reconfigureConfig, self.reconfig)
        self.wait=False
        # self.c_err = rospy.Publisher("c_err",Int32,queue_size=10)
        # self.a_err = rospy.Publisher("a_err",Int32,queue_size=10)
        # self.t_err = rospy.Publisher("t_err",Int32,queue_size=10)

    def callback(self,data):
        self.centre = data
        li = list(data.data.split(" "))
        cx = int(li[0])
        cy = int(li[1])
        mx = int(li[2])
        my = int(li[3])
        x1 = mx-cx
        y1 = my-cy
        mod = (x1**2+y1**2)**0.5
        deg = math.degrees(math.acos(x1/mod))
        self.bot_error(deg,cx,cy,x1,y1,mod)

    def bot_error(self,deg,cx,cy,x1,y1,mod):
        global list1
        if cy < list1[1][1]-30:
            m = (list1[1][1]-list1[0][1])/(list1[1][0]-list1[0][0])
            c = list1[0][1]-m*list1[0][0]
            err = (m*cx-cy+c)/(1+m**2)*0.5
            err_angle = 90 - math.degrees(math.acos(x1/mod))
            # print("angle error:",err_angle)
            # print("track error:",err)
            # print("total error",total_error)
            # self.c_error.pub(err)
            # self.a_error.pub(err_angle)
            self.wheel_speed(-100,err,err_angle)

        elif cy > list1[1][1]-30 :
            self.wheel_speed(0,0,0)

    def trim(self,err,limit):
        if err > limit:
            return limit
        if err < -limit:
            return -limit
        return err

    def wheel_speed(self,base,err,err_angle):
        base_speed = base
        total_err=w1*err+w2*err_angle
        err_speed=total_err
        # print("total_error: ",err_speed)
        # self.t_error.pub(total_error)

        err_speed=self.trim(err_speed,15)

        self.pub.publish(err_speed)
        # rs = int(base_speed + err_speed)
        # ls = int(base_speed - err_speed)
        rs = int(base_speed - err_speed)
        ls = int(base_speed + err_speed)    

        print("track error:",err," ls: ",ls ," rs ",rs)
        self.msg = str(ls)+","+str(rs)+","+"0"
        try :
            self.pub1.publish(self.msg)
        except rospy.ROSInterruptException as e:
            print(e)

    def reconfig(self,config,level):
        self.param = config
        print(self.param.kpa," ",self.param.base_speed)  
        return config              
#-------------------------------------------------------------------------bot turns-----------------------------------------
    def turn(self,s,i):
        ya=s[i][1]-s[i-1][1]
        yb=s[i][1]-s[i+1][1]
        xa=s[i][0]-s[i-1][0]
        xb=s[i][0]-s[i+1][0]
        r=180/math.pi*math.atan2(yb*xa-ya*xb, xa*xb+ya*yb)
        self.wait=True
        if abs(r-90)<15:
            #turn left
            while self.wait:
                pass
        elif abs(r+90)<15:
            #turn right
            while self.wait:
                pass
        elif abs(r)<15:
            #U turn
            while self.wait:
                pass

    def turnComplete(self,data):
        self.wait=False


    def left(self):
        try :
            self.pub1.publish("100,100,500")
        except rospy.ROSInterruptException as e:
            print(e)

    def turn(self,cx,cy,mx,my,fx,fy):
        pass


if __name__ == '__main__':
    obj = listen()
    try :
        rospy.spin()
    except Exception as e:
        print(e)
