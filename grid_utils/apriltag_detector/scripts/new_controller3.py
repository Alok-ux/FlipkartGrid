#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
import math
from dynamic_reconfigure.server import Server
from apriltag_detector.cfg import dynamic_reconfigureConfig

# list1= [[304,112],[278,361],[278,361]]
list1= [[278,373],[310,120],[278,373]]
# deg = 0
threshold=25
kpc=1.8
kpa=0
kic=0
kia=0
kdc=0.8
kda=0
prev_c=0
prev_a=0
I_c=0
I_a=0

class listen():
    def __init__(self):
        rospy.init_node("controller",anonymous=True)
        self.sub = rospy.Subscriber("apriltag_centre1",String,self.callback)
        self.ack = rospy.Subscriber("recv",String,self.turnComplete)
        self.pub1 = rospy.Publisher("wifi",String,queue_size=1)
        self.pub = rospy.Publisher("err",Int32,queue_size=1)
        self.pub3 = rospy.Publisher("correction",Int32,queue_size=1)
        srv = Server(dynamic_reconfigureConfig, self.reconfig)
        self.wait=False
        self.complete=False
        self.i=0
        self.speed_base=-95
        self.trimmer=15
        # self.degree=0
        # self.c_err = rospy.Publisher("c_err",Int32,queue_size=10)
        # self.a_err = rospy.Publisher("a_err",Int32,queue_size=10)
        # self.t_err = rospy.Publisher("t_err",Int32,queue_size=10)

    def callback(self,data):
        if self.complete or self.wait:
            return
        self.centre = data
        li = list(data.data.split(" "))
        cx = int(li[0])
        cy = int(li[1])
        mx = int(li[2])
        my = int(li[3])
        x1 = mx-cx
        y1 = my-cy
        mod = (x1**2+y1**2)**0.5
        # deg = math.degrees(math.acos(x1/mod))
        self.degree=math.degrees(math.atan2(y1,x1))
        self.bot_error_dist(cx,cy,x1,y1,mod)

    def bot_error_dist(self,cx,cy,x1,y1,mod):
        global list1
        distance=((list1[self.i+1][1]-cy)**2+(list1[self.i+1][0]-cx)**2)**0.5
        # if distance>threshold:
        m = (list1[self.i+1][1]-list1[self.i][1])/(list1[self.i+1][0]-list1[self.i][0])
        c = list1[self.i][1]-m*list1[self.i][0]
        err = (m*cx-cy+c)/(1+m*2)*0.5
        err_angle = 90 - math.degrees(math.acos(x1/mod))
        # print("angle error:",err_angle)
        # print("track error:",err)
        # print("total error",total_error)
        # self.c_error.pub(err)
        # self.a_error.pub(err_angle)
        self.wheel_speed(self.speed_base,err,err_angle)

        # else:
        #     self.wheel_speed(0,0,0)
        #     self.scheduler()

    # def bot_error(self,cx,cy,x1,y1,mod):
    #     global list1
    #     if cy < list1[self.i+1][1]-10:
    #         m = (list1[self.i+1][1]-list1[self.i][1])/(list1[self.i+1][0]-list1[self.i][0])
    #         c = list1[self.i][1]-m*list1[self.i][0]
    #         err = (m*cx-cy+c)/(1+m*2)*0.5
    #         err_angle = 90 - math.degrees(math.acos(x1/mod))
    #         # print("angle error:",err_angle)
    #         # print("track error:",err)
    #         # print("total error",total_error)
    #         # self.c_error.pub(err)
    #         # self.a_error.pub(err_angle)
    #         self.wheel_speed(self.speed_base,err,err_angle)

    #     elif cy > list1[self.i+1][1]-10 :
    #         self.wheel_speed(0,0,0)
    #         self.scheduler()
    #         # self.wait=True
    #         # self.arc_right()

    def trim(self,err,limit):
        if err > limit:
            return limit
        if err < -limit:
            return -limit
        return err

    def wheel_speed(self,base,err,err_angle):
        global deg,kpc,kpa,kic,kia,kdc,kda,prev_c,prev_a,I_c,I_a
        base_speed = base
        I_c=self.trim(I_c+err,5)
        total_err1=kpc*err+kdc*(err-prev_c)+kic*(I_c)
        prev_c=err
        I_a=self.trim(I_c+err_angle,5)
        total_err2=kpa*err_angle+kda*(err_angle-prev_a)+kic*(I_a)
        prev_a=err_angle
        err_speed=total_err1+total_err2
        # print("total_error: ",err_speed)
        # self.t_error.pub(total_error)

        err_speed=self.trim(err_speed,self.trimmer)
        if abs(self.degree-90)<20:
            err_speed=-err_speed
        self.pub.publish(int(err_angle))
        self.pub3.publish(int(err_speed))
        rs = int(base_speed + err_speed)
        ls = int(base_speed - err_speed)

        # rs = int(base_speed - err_speed)
        # ls = int(base_speed + err_speed)

        print("track error:",err," ls: ",ls ," rs ",rs)
        self.msg = str(ls)+","+str(rs)+","+"0"
        try :
            self.pub1.publish(self.msg)
        except rospy.ROSInterruptException as e:
            print(e)

    def reconfig(self,config,level):
        global deg,kpc,kpa,kic,kia,kdc,kda,prev_c,prev_a,I_c,I_a
        self.param = config
        print(self.param.kpc," ",self.param.kpa," ",self.param.base_speed)
        kpc=self.param.kpc
        kic=self.param.kic
        kdc=self.param.kdc
        kpa=self.param.kpa
        kia=self.param.kia
        kda=self.param.kda
        self.trimmer=self.param.trim_value
        self.speed_base=self.param.base_speed

        return config

#------------------------------------------------------------------------scheduler-----------------------------------------
    def scheduler(self):
        global list1
        print("scheduled ",self.i)
        self.i+=1
        self.wait=True
        if(self.i == len(list1)-1):
            self.complete=True
        else:
            self.turn()


#-------------------------------------------------------------------------bot turns-----------------------------------------
    def turn(self):
        global list1
        ya=list1[self.i][1]-list1[self.i-1][1]
        yb=list1[self.i][1]-list1[self.i+1][1]
        xa=list1[self.i][0]-list1[self.i-1][0]
        xb=list1[self.i][0]-list1[self.i+1][0]
        r=180/math.pi*math.atan2(yb*xa-ya*xb, xa*xb+ya*yb)
        if abs(r-90)<15:
            #turn left
            self.arc_left()
            pass
        elif abs(r+90)<15:
            #turn right
            self.arc_right()
            pass
        elif abs(r)<15:
            #U turn
            self.Uturn()
            pass

    def turnComplete(self,data):
        self.wait=False


    def left(self):
        try :
            self.pub1.publish("-120,120,505")
        except rospy.ROSInterruptException as e:
            print(e)
    def right(self):
        try :
            self.pub1.publish("120,-120,505")
        except rospy.ROSInterruptException as e:
            print(e)
    def Uturn(self):
        try :
            self.pub1.publish("120,-120,950")
        except rospy.ROSInterruptException as e:
            print(e)
    def arc_left(self):
        try :
            self.pub1.publish("0,240,500")
        except rospy.ROSInterruptException as e:
            print(e)
    def arc_right(self):
        try :
            self.pub1.publish("240,0,500")
        except rospy.ROSInterruptException as e:
            print(e)

    # def turn(self,cx,cy,mx,my,fx,fy):
    #     pass


if __name__ == '__main__':
    obj = listen()
    try :
        rospy.spin()
    except Exception as e:
        print(e)
