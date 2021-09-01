#!/usr/bin/env python

import rospy
import requests
from std_msgs.msg import String,Int32

z =1
class listen():
    def __init__(self):
        rospy.init_node("wifi_transmitter",anonymous=True)
        self.sub = rospy.Subscriber("wifi",String,self.callback)
        self.sub1 = rospy.Subscriber("bot_1",Int32,self.callback1)
        self.sub2 = rospy.Subscriber("bot_2",Int32,self.callback2)
        self.sub3 = rospy.Subscriber("bot_3",Int32,self.callback3)
        self.sub4 = rospy.Subscriber("bot_4",Int32,self.callback4)


    def callback(self,data):
        try:
            self.s = str(data)
            #print(self.s)
            if z == 1:
                url = 'http://192.168.43.193/'+self.s#check later
                #try:
                #    i = requests.get(url)
                #except Exception as e:
                #    print(e)

            elif z==2:
                url = 'http://192.168.43.193/'+self.s
                #try:
                #    i = requests.get(url)
                #except Exception as e:
                #    print(e)

            elif z==3:
                url = 'http://192.168.43.193/'+self.s
                #try:
                #    i = requests.get(url)
                #except Exception as e:
                #    print(e)

            elif z==4:
                url = 'http://192.168.43.193/'+self.s
                #try:
                #    i = requests.get(url)
                #except Exception as e:
                #    print(e)
        except rospy.ROSInterruptException as e:
            print(e)

    def callback1(self,data):
        global z
        try:
            z = data.data
            print(z)
        except ROSInterruptException as e:
            print(e)

    def callback2(self,data):
        global z
        try:
            z = data
        except ROSInterruptException as e:
            print(e)

    def callback3(self,data):
        global z
        try:
            z = data
        except ROSInterruptException as e:
            print(e)

    def callback4(self,data):
        global z
        try:
            z = data
        except ROSInterruptException as e:
            print(e)



if __name__ == '__main__':
    obj = listen()
    try:
        rospy.spin()
    except Exception as e:
        print(e)
