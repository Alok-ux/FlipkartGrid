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
        self.ack = rospy.Publisher("recv",String,queue_size=1)

    def callback(self,data):
        try:
            # msg = str(data.data)
            msg = "150,150,0"
            print(msg)
            self.ws.send(msg)
            self.ack.publish(self.ws.recv())
        except rospy.ROSInterruptException as e:
            print(e)
            self.ws.close()

if __name__ == '__main__':
    obj = listen("192.168.29.232:8888")
    try:
        rospy.spin()
    except Exception as e:
        print(e)
