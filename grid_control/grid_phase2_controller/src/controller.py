#!/usr/bin/env python3

import rospy
from std_msgs.msg import String,Int32
import math

kp_ang=0
ki_ang=0
kd_ang=0
kp_dist=0
ki_dist=0
kd_dist=0
prev_dist=0
prev_ang=0
I_dist=0
I_ang=0
act_dif_dist=5


class motion():
    def __init__(self):
        rospy.init_node("bot_motion",anonymous=True)
        self.sub1= rospy.Subscriber("apriltag pose from cam driver", String , self.callback_pose)
        self.sub2= rospy.Subscriber("Co-ordinates from client",String , self.callback_co_ord)
        self.pub=rospy.Publisher("dist",String ,queue_size=1)
        self.rate = rospy.Rate(10)

    def callback_pose(self,data):
        pose = list(data.data.split(" "))
        self.center = [pose[0], pose[1]]
        front=[pose[2], pose[3]]
        self.x1 = front[0]-center[0]
        self.y1 = front[1]-center[1]
        self.mod = (self.x1**2+self.y1**2)**0.5
        self.deg2= math.degrees(math.atan2(self.y1 , self.x1))



    def callback_co_ord(self,data):
        self.destination = list(data.data.split(" "))

    def move(self):
        x1=self.destnation[0]-self.center[0]
        y1=self.destnation[1]-self.center[1]
        dist= (x1**2+y1**2)**0.5
        deg1=math.degrees(math.atan2(y1 ,x1))
        ang= deg1-self.deg2
        if ang<-180:
            ang+=360
        if ang>180:
            ang-=360  
        final_speed=pid(dist , ang)
        vel=100 + final_speed[0]*5
        msg= str(vel)+str(Final_speed[1])
        self.pub.publish(msg)

    def pid(dist , ang):
        global kp_ang,ki_ang,kd_ang,kp_dist,ki_dist,kd_dist,prev_dist,prev_ang,I_dist,I_ang
        err_ang = ang
        err_dist = dist
        diff_dist = err_dist - I_dist
        lim_dist = act_dif_dist - diff_dist
        diff_ang = err_ang -I_ang
        final_dist= kp*lim_dist + ki*lim_dist
        final_ang= kp*err_ang + ki*err_ang + kd*diff_ang
        I_dist = err_dist
        I_ang = err_ang
        speed= [final_dist , final_ang]

        return speed
if __name__ == '__main__':
    obj = motion()
    try :
        rospy.spin()
    except Exception as e:
        print(e)
