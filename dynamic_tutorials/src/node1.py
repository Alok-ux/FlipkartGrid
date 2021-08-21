#!/usr/bin/env python
import rospy
import cv2
import apriltag
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray,String
from cv_bridge import CvBridge,CvBridgeError


class listener():
    def __init__(self):
        rospy.init_node('listener',anonymous = True)
        self.sub = rospy.Subscriber("/overhead_camera/image_raw",Image,self.callback)
        self.pub = rospy.Publisher("apriltag_centre",String,queue_size = 10)
        self.rate = rospy.Rate(30)
        self.bridge = CvBridge()

    def callback(self,data):
        try :
            self.image = self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.gray = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)

            options = apriltag.DetectorOptions('tag36h11')
            detector = apriltag.Detector(options)
            results = detector.detect(self.gray)

            for i in results:
                pta , ptb ,ptc ,ptd = i.corners
                cn = i.center
                pta = (int(pta[0]),int(pta[1]))
                ptb = (int(ptb[0]),int(ptb[1]))
                ptc = (int(ptc[0]),int(ptc[1]))
                ptd = (int(ptd[0]),int(ptd[1]))
                pte = (int((pta[0]+ptb[0])/2),int((pta[1]+ptb[1])/2))
                cn = (int(cn[0]),int(cn[1]))
                self.str1 = "("+str(cn[0])+","+str(cn[1])+")"
                self.str2 = "("+str(pte[0])+","+str(pte[1])+")"
                self.stri = self.str1+self.str2

                self.publ(self.stri)

                cv2.rectangle(self.image,pta,ptc,(0,255,0),2)
                cv2.circle(self.image,cn,3,(0,0,255),-1)
                cv2.circle(self.image,pte,3,(0,0,255),-1)

            # cv2.imshow("image",self.image)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
        except CvBridgeError as e:
            print(e)

    def publ(self,arg):
        while not rospy.is_shutdown():
            try:
                self.pub.publish(arg)
                self.rate.sleep()
                break
            except rospy.ROSInterruptException as e:
                print(e)
                break



if __name__ == '__main__':
    obj = listener()
    try:
        rospy.spin()
        print("entering2")
    except Exception as e:
        print(e)
