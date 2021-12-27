#!/usr/bin/env python


import websocket
import rospy

from std_msgs.msg import String,Int32
class listen():
    def __init__(self,host):
        rospy.init_node("wifi_transmitter",anonymous=True)
        self.ws= websocket.WebSocket()
        self.host = host
        self.ws.connect("ws://"+self.host)
        self.sub = rospy.Subscriber("wifi3",String,self.callback)
        # self.sub2 = rospy.Subscriber("id",Int32,self.callback2)
        self.ack = rospy.Publisher("recv",String,queue_size=1)
        

    def callback(self,data):
        try:
            msg = str(data.data)
            print(msg)
            self.ws.send(msg)
            self.ack.publish(self.ws.recv())
        except rospy.ROSInterruptException as e:
            print(e)
            self.ws.close()

    # def callback2(self,data):
    #     print("inside callback")
    #     print(data.data)
    #     if(data.data==1):
    #         self.ws="ws://"+"192.168.29.80:8888"   
    #     if(data.data==2):
    #         self.ws="ws://"+"192.168.29.105:8888"
    #     if(data.data==3):
    #         self.ws="ws://"+"192.168.29.244:8888"      
    #     if(data.data==4):
    #         self.ws="ws://"+"192.168.29.244:8888"    

if __name__ == '__main__':
    obj = listen("192.168.29.244:8888")
    try:
        rospy.spin()
    except Exception as e:
        print(e)

