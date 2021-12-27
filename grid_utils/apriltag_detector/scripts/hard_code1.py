#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
import math
from dynamic_reconfigure.server import Server
from apriltag_detector.cfg import dynamic_reconfigureConfig

# list1= [[304,112],[278,361],[278,361]]
list1= [[286,100],[266,370],[286,100]]
deg = 0
kpc=1
kpa=0
kic=0
kia=0
kdc=0
kda=0
prev_c=0
prev_a=0
I_c=0
I_a=0

class Bot():
    def __init__(self):
        rospy.init_node("controller",anonymous=True)
        # self.sub = rospy.Subscriber("apriltag_centre1",String,self.callback)
        self.ack = rospy.Subscriber("recv",String,self.turnComplete)
        self.hack = rospy.Subscriber("hardcode",String,self.wheel_speed)
        self.pub1 = rospy.Publisher("wifi",String,queue_size=1)
        # self.pub = rospy.Publisher("err",Int32,queue_size=1)
        # self.pub3 = rospy.Publisher("correction",Int32,queue_size=1)
        # srv = Server(dynamic_reconfigureConfig, self.reconfig)
        self.wait=False
        self.complete=False
        self.i=0
        self.speed_base=95
        self.trimmer=15
        # rospy.spinOnce()
        # self.wheel_speed()
        # self.c_err = rospy.Publisher("c_err",Int32,queue_size=10)
        # self.a_err = rospy.Publisher("a_err",Int32,queue_size=10)
        # self.t_err = rospy.Publisher("t_err",Int32,queue_size=10)

    def trim(self,err,limit):
        if err > limit:
            return limit
        if err < -limit:
            return -limit
        return err

    def wheel_speed(self,data):
        print("inside wheel speed")
        if(not self.complete):
          if(self.i==0):
            self.forward(3000)
          if(self.i==1):
            self.pub1.publish("0,0,400")
          if(self.i==2):
            self.right()
          if(self.i==3):
            self.forward(2500) 
          if(self.i==4):
            self.Uturn()        
                               

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

    def forward(self,tim=3000):
        print("inside forward speed")
        try :
            self.pub1.publish("123,120,"+str(tim))
        except rospy.ROSInterruptException as e:
            print(e)   

    def turnComplete(self,data):
        print("turn completed")
        self.i+=1
        self.wheel_speed("")

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
    obj = Bot()
    # obj.wheel_speed()
    try :
        rospy.spin()
    except Exception as e:
        print(e)
