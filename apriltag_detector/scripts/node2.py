#!/usr/bin/env python
#author: Debanshu
#token: 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
import rospy
import re
import math
from std_msgs.msg import Int32,String
from geometry_msgs.msg import Twist



list1 = [[542,87],[542,602],[85,602]]
list2 = [[604,87],[604,666],[85,666]]
list3 = [[731,87],[731,666],[1186,666]]
list4 = [[667,87],[667,602],[1186,602]]
p,i,d = 1,0.01,0.01
flag1 , flag2 , flag3 ,flag4 ,flag5 ,flag6,flag7,flag8 = False,False,False,False,False,False,False,False
g = None
class listen():

    def __init__(self):
        rospy.init_node('listener2',anonymous=True)
        self.sub = rospy.Subscriber("apriltag_centre1",String,self.callback)
        self.pub = rospy.Publisher("wifi",String,queue_size=10)
        self.pub1 = rospy.Publisher("bot_a",Int32,queue_size=10)
        self.pubn = rospy.Publisher("bot_no_1",Int32,queue_size=10)
        self.msg = Twist()
        self.rate = rospy.Rate(30)


    def callback(self,data):
        global g
        try:
            self.centre = str(data)
            obj = re.compile(r'\((\d{1,4}),(\d{1,4})\)')
            mo = list(map(list,obj.findall(self.centre)))
            for i in range(len(mo)):
                mo[i] = list(map(int,mo[i]))


            x1, y1 = mo[1][0]-mo[0][0],mo[1][1]-mo[0][1]
            mod1 = (x1**2+y1**2)**0.5
            deg = math.degrees(math.acos(x1/mod1))

            if g == None:
                # print(err_deg)
                g=self.bot_error(deg,x1,y1,*mo[0])
            elif g == True:
                self.rev_bot_error(deg,x1,y1,*mo[0])

            # print("entering1")
            # print(mo)
        except rospy.ROSInterruptException as e:
            print(e)

    def wheel_speed(self,base,err,err_ang):
        base_speed = base
        err_speed = p*(err-err_ang)
        rd = int(base_speed+err_speed)
        ld = int(base_speed-err_speed)
        self.msg = str(rd)+str(ld)+'0'
        try:
            self.pub.publish(self.msg)
            self.rate.sleep()

        except rospy.ROSInterruptException as e:
            print(e)

    def bot_error(self,deg,x1,y1,*arg2):
        arg2 = list(arg2)
        global list1,p,i,d,flag1,flag3,flag4
        if arg2[1] < list1[1][1] and flag1 == False:
            print("in 1")
            err = list1[1][0]-arg2[0]
            mod1 = (x1**2+y1**2)**0.5
            err_deg = 90 - math.degrees(math.acos(x1/mod1))
            self.wheel_speed(100,err,err_deg)

        elif arg2[1] >= list1[1][1]-30 and deg <= 178 and flag1 == False :
            print("in 2")
            self.wheel_speed(100,0,100)
            flag1 = True


        elif arg2[0] < list1[2][0] :
            print("in 4")
            self.wheel_speed(0,0,0)
            return True



        elif arg2[1] >= list1[1][1]-66 and deg >178 and flag1 == True and flag4 == False:
            flag3 = True
            flag4 = True
        elif arg2[1] >= list1[1][1]-66 and flag3 == True and flag1 == True:
            print("in 3")
            err = list1[1][1]-arg2[1]
            mod1 = (x1**2+y1**2)**0.5
            err_deg = 90 - math.degrees(math.acos(y1/mod1))
            self.wheel_speed(100,err,err_deg)

    def rev_bot_error(self,deg,x1,y1,*arg2):
        arg2 = list(arg2)
        global list1,p,i,d,flag2,flag5,flag6,flag7,flag8
        if arg2[0] <= list1[2][0]+20 and flag2 == False:
            print("in 5")
            self.wheel_speed(100,0,100)
            flag2 = True
            return True


        elif arg2[0] <= list1[1][0]-10 and flag2 == True and deg <= 2 and flag6 == False:
            flag5 = True
            flag6 = True

        elif arg2[0] <= list1[1][0]-10 and flag2 == True and flag5 == True:
            print("in 6")
            err = arg2[1]-list1[1][1]
            mod2 = (x1**2+y1**2)**0.5
            err_deg = math.degrees(math.acos(y1/mod2))-90
            self.wheel_speed(100,err,err_deg)

        elif arg2[0] > list1[1][0]-10 and flag2 == True and deg < 10 :
            self.wheel_speed(100,100,0)
            flag2 = True
            return True


        # elif arg2[1] > list1[1][0]-10 and deg > 88 and flag7 == False and flag2 == True:
        #     flag7 = True
        #     flag8 = True
        elif arg2[1] < list1[0][1]:
            print("in 8")
            flag2 = True
            try:
                self.wheel_speed(0,0,0)
                self.pub1.publish(1)
                self.pubn.publish(1)
                self.rate.sleep()
                exit()
                return True

            except rospy.ROSInterruptException as e:
                print(e)


        elif arg2[0] > list1[1][0]-66 and flag2 == True and deg > 80:
            print("in 7")
            err = arg2[0]-list1[1][0]
            mod2 = (x1**2+y1**2)**0.5
            err_deg = math.degrees(math.acos(x1/mod2))-90
            self.wheel_speed(100,err,err_deg)





if __name__=='__main__':
    obj = listen()
    try:
        rospy.spin()
    except Exception as e:
        print(e)
