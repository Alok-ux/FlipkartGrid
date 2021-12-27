#!/usr/bin/env python3


import websocket
import rospy

from std_msgs.msg import String,Int32
class listen():
    def __init__(self,host):
        rospy.init_node("wifi_transmitter",anonymous=True)
        self.ws= websocket.WebSocket()
        self.ws.connect("ws://"+host)
        self.sub = rospy.Subscriber("wifi",String,self.callback)

    def callback(self,data):
        try:
            msg = str(data.data)
            print(msg)
            self.ws.send(msg)
            print(self.ws.recv())
        except rospy.ROSInterruptException as e:
            print(e)
            self.ws.close()

if __name__ == '__main__':
    obj = listen("192.168.29.105:8888")
    try:
        rospy.spin()
    except Exception as e:
        print(e)
