#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
import math
from dynamic_reconfigure.server import Server
from apriltag_detector.cfg import dynamic_reconfigureConfig
import cv2
import numpy as np


class Bot():
    def __init__(self):
        rospy.init_node("controller",anonymous=True)
        self.ack = rospy.Subscriber("recv",String,self.scheduler)
        self.st = rospy.Subscriber("start",Int32,self.start)
        self.pub1 = rospy.Publisher("wifi4",String,queue_size=1)
        self.pub_id= rospy.Publisher("id",Int32,queue_size=1)
        self.i=0

        self.wait=False
        self.complete=False
        self.speed_base=95
        self.trimmer=15
       

    def start(self,data):
        # self.i=0
        # self.scheduler("")       
        while True:
            blank = np.zeros((200,200,3), np.uint8)
            cv2.imshow("bot4", blank)
		
            key_pressed = cv2.waitKey(0)
            self.pub_id.publish(4)

            if key_pressed == ord('x'):
                self.forward(500)

            elif key_pressed == ord('a'):
                self.left()

            elif key_pressed == ord('d'):
                self.right()

            elif key_pressed == ord('q'):
                self.left(100)

            elif key_pressed == ord('e'):
                self.right(100)                

            elif key_pressed == ord('s'):
                self.waitms()

            elif key_pressed == ord('u'):
                self.Uturn()

            elif key_pressed == ord('w'):
                self.back()

            elif key_pressed == ord('z'):
                self.drop()                

            elif key_pressed == ord('b'):
                break

            print(key_pressed)   
                       

    def scheduler(self,data):
        # print("schedulling move :",self.i)
        # if(self.i==0):
        #     self.forward(1000)
        # if(self.i==1):
        #     self.waitms()            
        # if(self.i==2):
        #     self.forward(1000)            
        # if(self.i==3):
        #     self.waitms()
        # if(self.i==4):
        #     self.right()
        # if(self.i==5):
        #     self.waitms()            
        # if(self.i==6):
        #     self.forward(1500) 
        # if(self.i==6):
        #     self.waitms()              
        # if(self.i==7):
        #     self.drop()     
        # if(self.i==8):
        #     self.waitms() 

            
                                            
        # if(self.i==9):
        #     self.Uturn()
        # if(self.i==10):
        #     self.forward(1500)            
        # if(self.i==11):
        #     self.waitms()       
        # if(self.i==12):
        #     self.left()
        # if(self.i==13):
        #     self.waitms()            
        # if(self.i==14):
        #     self.forward(1500)                  
        self.i+=1




    def forward(self,tim=3000):
        print("inside forward speed")
        try :
            self.pub1.publish("120,120,"+str(tim))
        except rospy.ROSInterruptException as e:
            print(e)   

    def drop(self):
        try :
            self.pub1.publish("0,0,1")
        except rospy.ROSInterruptException as e:
            print(e) 

    def waitms(self,tim=200):
        try :
            self.pub1.publish("0,0,"+str(tim))
        except rospy.ROSInterruptException as e:
            print(e)

    def left(self,tym=510):
        try :
            self.pub1.publish("-120,120,"+str(tym))
        except rospy.ROSInterruptException as e:
            print(e)
    def right(self,tym=510):
        try :
            self.pub1.publish("120,-120,"+str(tym))
        except rospy.ROSInterruptException as e:
            print(e)    
    def back(self):
        try :
            self.pub1.publish("-120,-120,500")
        except rospy.ROSInterruptException as e:
            print(e)                

    # def slow_left(self):
    #     try :
    #         self.pub1.publish("-120,120,505")
    #     except rospy.ROSInterruptException as e:
    #         print(e)
    # def slow_right(self):
    #     try :
    #         self.pub1.publish("120,-120,510")
    #     except rospy.ROSInterruptException as e:
    #         print(e)  

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



if __name__ == '__main__':
    obj = Bot()

    try :
        rospy.spin()
    except Exception as e:
        print(e)

