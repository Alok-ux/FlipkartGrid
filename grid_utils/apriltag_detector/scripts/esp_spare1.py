#!/usr/bin/env python3

import rospy
import telnetlib
from std_msgs.msg import String,Int32
class listen():
    def __init__(self):
        rospy.init_node("wifi_transmitter",anonymous=True)
        self.sub = rospy.Subscriber("wifi",String,self.callback)

    def callback(self,data):
        try:
            self.s = data.data
            print(self.s)
            HOST = "192.168.29.105" #enter ip address
            port = "8888" #enter port no
            tn = telnetlib.Telnet(HOST, port)
            # msg = "{},{},0\r".format(int(data.left_rpm*10), int(data.right_rpm*10))
            msg = str(self.s+"\r")#string of pwm values
            tn.write(msg.encode('ascii'))
            tn.close()

        except rospy.ROSInterruptException as e:
            print(e)

if __name__ == '__main__':
    obj = listen()
    try:
        rospy.spin()
    except Exception as e:
        print(e)
