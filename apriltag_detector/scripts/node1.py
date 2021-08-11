#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError


class listener():
    def __init__(self):
        rospy.init_node('listener',anonymous = True)
        self.sub = rospy.Subscriber("/overhead_camera/image_raw",Image,self.callback)
        #self.rate = rospy.Rate()
        self.bridge = CvBridge()

    def callback(self,data):
        try :
            self.image = self.bridge.imgmsg_to_cv2(data,"bgr8")
            cv2.imshow("image",self.image)
            cv2.waitKey(34)
            cv2.destroyAllWindows()
        except CvBridgeError as e:
            print(e)



if __name__ == '__main__':
    obj = listener()
    try:
        rospy.spin()
        print("entering2")
    except Exception as e:
        print(e)
