#!/usr/bin/env python

import rospy
from std_msgs.msg import String,Int32
import math
from dynamic_reconfigure.server import Server
from apriltag_detector.cfg import dynamic_reconfigureConfig

list1 = [[270,15],[270,341],[270,15]]
deg = 0
w1=1
w2=0

class listen():
    def __init__(self):
        rospy.init_node("controller",anonymous=True)
        self.sub = rospy.Subscriber("apriltag_centre1",String,self.callback)
        self.pub1 = rospy.Publisher("wifi",String,queue_size=1)
        srv = Server(dynamic_reconfigureConfig, self.reconfig)
        self.pub2 = rospy.Publisher("err",Int32,queue_size=1)
        self.rate = rospy.Rate(300)

    def callback(self,data):
        self.centre = data
        li = list(self.centre.split(" "))
        cx = int(li[0])
        cy = int(li[1])
        mx = int(li[2])
        my = int(li[3])
        x1 = mx-cx
        y1 = my-cy
        mod = (x1**2+y1**2)**0.5

        deg = math.degress(math.acos(x1/mod))
        self.bot_error(deg,cx,cy,x1,y1,mod)

    def bot_error(self,deg,cx,cy,x1,y1,mod):
        global list1
        if cy < list1[1][1]-30:
            m = (list1[1][1]-list1[0][1])/(list1[1][0]-list1[0][0])
            c = list1[0][1]-m*list1[0][0]
            err = (m*cx-cy+c)/(1+m**2)**0.5
            err_angle = 90 - math.degress(math.acos(x1/mod))
            print("track_err ",err," err_angle ",err_angle)

            self.wheel_speed(100,err,err_angle)

        elif cy > list1[1][1]-30 :
            self.wheel_speed(0,0,0)
    
    def wheel_speed(self,base,err,err_angle):
        base_speed = base
        total_error= w1*err+w2*err_angle
        err_speed=total_error
        print("track_err ",err," err_angle ",err_angle," total_err ",total_error)
        self.pub2.publish(int(err_speed))
        rs = int(base_speed +err_speed)
        ls = int(base_speed - err_speed)
        self.msg = str(ls)+","+str(rs)+","+"0"
        try :
            self.pub1.publish(self.msg)
            self.pub.publish(err)
            self.rate.sleep()
        except rospy.ROSInterruptException as e:
            print(e)

    def reconfig(self,config,level):
        self.param = config
        print(self.param.kpa," ",self.param.base_speed)  
        return config      

if __name__ == '__main__':
    obj = listen()
    try :
        rospy.spin()
    except Exception as e:
        print(e)
