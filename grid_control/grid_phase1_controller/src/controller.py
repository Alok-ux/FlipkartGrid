#!/usr/bin/env python3

import cv2
import rospy
import math
import apriltag
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from differential_drive.msg import DiffRPM

from dynamic_reconfigure.server import Server
from grid_phase1_controller.cfg import PidConfig
from grid_array.srv import GridArray, GridArrayRequest

class Controller:
    def __init__(self, args):
        rospy.init_node("controller")
        srv = Server(PidConfig, self.dyn_callback)

        self.pub = rospy.Publisher(args.rpm_topic, DiffRPM, queue_size=1)
        self.msg = DiffRPM()

        self.bridge = CvBridge()
        self.image = None
        self.tag_id = args.tag_id
        self.intg, self.last_error = 0.0, 0.0
        self.params = dict()
        self.bot, self.stage = 0, 0

        self.goal_array = [[(100, 100), (200, 200), (300, 300), (400, 400)],
                           [(100, 100), (200, 200), (300, 300), (400, 400)],
                           [(100, 100), (200, 200), (300, 300), (400, 400)],
                           [(100, 100), (200, 200), (300, 300), (400, 400)]]

        rospy.loginfo('waiting for %s server', args.srv_name)
        rospy.wait_for_service(args.srv_name)
        rospy.loginfo('waiting for %s topic', args.image_topic)

        req = GridArrayRequest()
        req.image = rospy.wait_for_message(args.image_topic, Image)
        try:
            service = rospy.ServiceProxy(args.srv_name, GridArray)
            self.goal_array = service(req)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)

        rospy.Subscriber(args.image_topic, Image, self.callback)
        rospy.loginfo("apriltag detecting tag_id: %s", self.tag_id)

    def dyn_callback(self, config, level):
        print(config)
        self.params = config
        return config

    def callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
            options = apriltag.DetectorOptions()
            detector = apriltag.Detector(options)
            results = detector.detect(gray)

            for i in results:
                # if i.tag_id == self.tag_id:
                corners = [(int(x), int(y)) for (x, y) in i.corners]
                center = tuple([int(x) for x in i.center])
                mid = ((corners[0][0] + corners[1][0]) // 2,
                       (corners[0][1] + corners[1][1]) // 2)
                direction = math.atan2(center[1]-mid[1], -center[0]+mid[0])

                init = self.goal_array[self.bot][self.stage - 1]
                final = self.goal_array[self.bot][self.stage]
                path_angle = math.atan2(final[1]-init[1], final[0]-init[0])

                cv2.arrowedLine(self.image, init, final, (0, 255, 0), 2)
                print(center, mid)
                cv2.arrowedLine(self.image, center, mid, (255, 255, 0), 2)
                for i, c in enumerate(corners):
                    cv2.putText(self.image, str(i+1), c, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

                # self.angular_controller(center, direction, (init, final), path_angle)
                self.linear_controller(center, direction, (init, final), path_angle)

            cv2.imshow("frame", self.image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown('keyboard interrupt')
        except CvBridgeError as e:
            rospy.loginfo(e)

    def pid(self, error):
        prop = error
        self.intg = self.intg + error if self.intg < 255 else 0
        diff = error - self.last_error
        self.last_error = error
        return self.params['KP'] * prop + self.params['KI'] * self.intg + self.params['KD'] * diff

    def linear_controller(self, bot_pos, bot_dir, path_pos, path_dir):
        (x1, y1), (x2, y2) = path_pos
        xc, yc = bot_pos
        m1 = math.tan(path_dir)
        m2 = -1 / m1
        x3 = int((m2 * xc - m1 * x1 + y1 - yc) / (m2 - m1))
        y3 = int(m2 * (x3 - xc) + yc)

        cv2.line(self.image, bot_pos, (x3, y3), (0, 255, 0), 2)

        cross_track_error = math.dist(bot_pos, (x3, y3))
        # balance = self.pid(cross_track_error)

        # self.msg.left_rpm = self.params['BS'] + balance
        # self.msg.right_rpm = self.params['BS'] - balance
        # self.pub.publish(self.msg)

    def angular_controller(self, bot_pos, bot_dir, path_pos, path_dir):
        angular_error = path_dir - bot_dir
        cv2.ellipse(self.image, bot_pos, 1, 0, bot_dir, path_dir, (255, 0, 0), 2)
        balance = self.pid(angular_error)

        self.msg.left_rpm = self.params['BS'] + balance
        self.msg.right_rpm = self.params['BS'] - balance
        self.pub.publish(self.msg)


if __name__ == '__main__':
    parser = argparse.ArgumentParser('grid_phase1_controller')
    parser.add_argument('tag_id', type=int, default=1,
                        help='robot tag id, default: 1')
    parser.add_argument('--image_topic', type=str,
                        default='/overhead_camera/image_raw',
                        help='camera topic, default: /overhead_camera/image_raw')
    parser.add_argument('--rpm_topic', type=str,
                        default='/grid_robot/rpm',
                        help='rpm topic, default: /grid_robot/rpm')
    parser.add_argument('--srv_name', type=str,
                        default='get_grid_array',
                        help='srv name, default: get_grid_array')

    args = parser.parse_args()
    controller = Controller(args)
    try:
        rospy.spin()
    except ROSInterruptException as e:
        rospy.loginfo(e)
