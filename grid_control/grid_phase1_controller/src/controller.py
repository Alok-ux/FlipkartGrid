#!/usr/bin/env python3

import cv2
import rospy
import math
import apriltag
import argparse

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from grid_transmitter.msg import PwmCombined

from dynamic_reconfigure.server import Server
from grid_phase1_controller.cfg import PidConfig
# from grid_array.srv import GridArray, GridArrayRequest

class Controller:
    def __init__(self, args):
        rospy.init_node("controller")
        self.msg_rpm = PwmCombined()
        self.intg, self.last_error = 0.0, 0.0

        self.bot, self.stage = -1, 0
        self.change_bot()

        self.goal_array = [[(100, 100), (200, 200), (300, 300), (400, 400)],
                           [(100, 100), (200, 200), (300, 300), (400, 400)],
                           [(100, 100), (200, 200), (300, 300), (400, 400)],
                           [(100, 100), (200, 200), (300, 300), (400, 400)]]

        # rospy.loginfo('waiting for %s server', args.srv_name)
        # rospy.wait_for_service(args.srv_name)
        # rospy.loginfo('waiting for %s topic', args.image_topic)
        #
        # req = GridArrayRequest()
        # req.image = rospy.wait_for_message(args.image_topic, Image)
        # try:
        #     service = rospy.ServiceProxy(args.srv_name, GridArray)
        #     self.goal_array = service(req)
        # except rospy.ServiceException as e:
        #     rospy.loginfo("Service call failed: %s", e)

        self.params = dict()
        srv = Server(PidConfig, self.dyn_callback)

        self.bridge = CvBridge()
        self.image = None
        rospy.Subscriber(args.image_topic, Image, self.callback)
        rospy.loginfo("Mission Started...")

    def change_bot(self):
        if self.bot == 3:
            rospy.loginfo("Mission Completed")
            rospy.signal_shutdown("Mission Completed")
        self.bot += 1
        self.pub = rospy.Publisher('{}_{}'.format(args.rpm_topic, self.bot), PwmCombined, queue_size=1)

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
                if i.tag_id == self.bot + 1:
                    corners = [(int(x), int(y)) for (x, y) in i.corners]
                    center = tuple([int(x) for x in i.center])
                    mid = ((corners[0][0] + corners[1][0]) // 2,
                           (corners[0][1] + corners[1][1]) // 2)
                    direction = math.atan2(center[1]-mid[1], -center[0]+mid[0])

                    init = self.goal_array[self.bot][self.stage - 1]
                    final = self.goal_array[self.bot][self.stage]
                    path_angle = math.atan2(final[1]-init[1], final[0]-init[0])

                    cv2.arrowedLine(self.image, init, final, (0, 255, 0), 2)
                    cv2.arrowedLine(self.image, center, mid, (255, 255, 0), 2)
                    for i, c in enumerate(corners):
                        cv2.putText(self.image, str(i+1), c, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

                    # self.angular_controller(center, direction, (init, final), path_angle)
                    self.linear_controller(init, path_angle, center)

            cv2.imshow("frame", self.image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown('Mission Aborted')
        except CvBridgeError as e:
            rospy.loginfo(e)

    def pid(self, error):
        prop = error
        self.intg = self.intg + error if self.intg < 255 else 0
        diff = error - self.last_error
        self.last_error = error
        return self.params['KP'] * prop + self.params['KI'] * self.intg + self.params['KD'] * diff

    def linear_controller(self, path_pos, path_angle, bot_pos):
        x1, y1 = path_pos
        xc, yc = bot_pos
        m = math.tan(path_angle)
        dist = (m * (xc - x1) - (yc - y1)) / (m ** 2 + 1) ** 0.5
        dist = -dist if math.cos(angle) < 0 else dist
        balance = self.pid(dist)

        cv2.circle(self.image, bot_pos, abs(int(dist)), (255, 0, 0), 2)
        self.msg.left = self.params['BS'] + balance
        self.msg.right = self.params['BS'] - balance
        self.pub.publish(self.msg)

    def angular_controller(self, bot_pos, bot_dir, path_pos, path_dir):
        angular_error = path_dir - bot_dir
        balance = self.pid(angular_error)

        self.msg.left = self.params['BS'] + balance
        self.msg.right = self.params['BS'] - balance
        self.pub.publish(self.msg)


if __name__ == '__main__':
    parser = argparse.ArgumentParser('grid_phase1_controller')
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
