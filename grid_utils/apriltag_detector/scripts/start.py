#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Int32

def talker():
    pub = rospy.Publisher('start', Int32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    
    # hello_str = "100 100 120 100"
    # msg = "123,120,3000"
    pub.publish(1)
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
