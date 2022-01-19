#! /usr/bin/env python3

import cv2
import time
import math
import rospy
import argparse
import actionlib

from cv_bridge import CvBridge, CvBridgeError
from grid_phase2_controller.msg import botAction, botFeedback, botResult
from camera_driver.msg import GridPoseArray
from grid_transmitter.msg import PwmCombined

cordinates = {
    (0, 4): [58, 343],
    (0, 9): [68, 170],
    (1, 1): [89, 452],
    (1, 2): [90, 415],
    (1, 3): [92, 379],
    (1, 4): [93, 343],
    (1, 5): [95, 308],
    (1, 6): [97, 272],
    (1, 7): [98, 237],
    (1, 8): [99, 203],
    (1, 9): [101, 169],
    (1, 10): [102, 136],
    (1, 11): [104, 103],
    (1, 12): [105, 71],
    (1, 13): [107, 40],
    (2, 1): [126, 453],
    (2, 2): [127, 416],
    (2, 3): [128, 379],
    (2, 4): [129, 344],
    (2, 5): [131, 308],
    (2, 6): [132, 273],
    (2, 7): [134, 238],
    (2, 8): [135, 204],
    (2, 9): [136, 170],
    (2, 10): [137, 136],
    (2, 11): [138, 104],
    (2, 12): [139, 71],
    (2, 13): [141, 40],
    (3, 1): [162, 453],
    (3, 2): [163, 416],
    (3, 3): [164, 379],
    (3, 4): [166, 343],
    (3, 5): [167, 308],
    (3, 6): [168, 274],
    (3, 7): [169, 237],
    (3, 8): [170, 204],
    (3, 9): [171, 170],
    (3, 10): [172, 136],
    (3, 11): [172, 104],
    (3, 12): [173, 72],
    (3, 13): [175, 40],
    (4, 1): [199, 453],
    (4, 2): [200, 416],
    (4, 3): [200, 379],
    (4, 4): [202, 343],
    (4, 5): [203, 308],
    (4, 6): [200, 274],
    (4, 7): [200, 237],
    (4, 8): [205, 205],
    (4, 9): [205, 171],
    (4, 10): [205, 136],
    (4, 11): [205, 103],
    (4, 12): [207, 72],
    (4, 13): [208, 41],
    (5, 1): [236, 453],
    (5, 2): [236, 416],
    (5, 3): [237, 380],
    (5, 4): [237, 344],
    (5, 5): [238, 308],
    (5, 6): [238, 274],
    (5, 7): [239, 239],
    (5, 8): [240, 205],
    (5, 9): [240, 172],
    (5, 10): [240, 138],
    (5, 11): [241, 106],
    (5, 12): [241, 73],
    (5, 13): [242, 41],
    (6, 1): [272, 453],
    (6, 2): [272, 416],
    (6, 3): [273, 380],
    (6, 4): [273, 344],
    (6, 5): [273, 309],
    (6, 6): [274, 274],
    (6, 7): [274, 240],
    (6, 8): [274, 206],
    (6, 9): [274, 172],
    (6, 10): [275, 139],
    (6, 11): [275, 106],
    (6, 12): [275, 74],
    (6, 13): [275, 42],
    (7, 1): [308, 453],
    (7, 2): [308, 416],
    (7, 3): [308, 380],
    (7, 4): [308, 345],
    (7, 5): [309, 309],
    (7, 6): [309, 274],
    (7, 7): [309, 240],
    (7, 8): [309, 206],
    (7, 9): [309, 173],
    (7, 10): [309, 139],
    (7, 11): [309, 106],
    (7, 12): [309, 74],
    (7, 13): [309, 43],
    (8, 1): [344, 453],
    (8, 2): [344, 416],
    (8, 3): [344, 380],
    (8, 4): [344, 345],
    (8, 5): [344, 309],
    (8, 6): [344, 274],
    (8, 7): [344, 240],
    (8, 8): [343, 207],
    (8, 9): [343, 173],
    (8, 10): [343, 139],
    (8, 11): [343, 106],
    (8, 12): [342, 75],
    (8, 13): [342, 44],
    (9, 1): [380, 453],
    (9, 2): [379, 416],
    (9, 3): [379, 381],
    (9, 4): [379, 345],
    (9, 5): [378, 310],
    (9, 6): [378, 275],
    (9, 7): [378, 241],
    (9, 8): [377, 207],
    (9, 9): [377, 174],
    (9, 10): [376, 141],
    (9, 11): [376, 108],
    (9, 12): [376, 76],
    (9, 13): [376, 44],
    (10, 1): [415, 453],
    (10, 2): [415, 416],
    (10, 3): [414, 381],
    (10, 4): [414, 345],
    (10, 5): [413, 310],
    (10, 6): [412, 275],
    (10, 7): [412, 242],
    (10, 8): [411, 208],
    (10, 9): [411, 174],
    (10, 10): [410, 142],
    (10, 11): [410, 109],
    (10, 12): [409, 77],
    (10, 13): [409, 45],
    (11, 1): [451, 453],
    (11, 2): [451, 416],
    (11, 3): [451, 381],
    (11, 4): [448, 345],
    (11, 5): [448, 310],
    (11, 6): [448, 275],
    (11, 7): [448, 242],
    (11, 8): [446, 208],
    (11, 9): [445, 175],
    (11, 10): [445, 142],
    (11, 11): [445, 109],
    (11, 12): [442, 77],
    (11, 13): [441, 46],
    (12, 1): [486, 453],
    (12, 2): [486, 416],
    (12, 3): [486, 381],
    (12, 4): [483, 346],
    (12, 5): [482, 311],
    (12, 6): [482, 275],
    (12, 7): [482, 242],
    (12, 8): [479, 209],
    (12, 9): [478, 176],
    (12, 10): [478, 142],
    (12, 11): [478, 109],
    (12, 12): [475, 79],
    (12, 13): [474, 48],
    (13, 1): [520, 452],
    (13, 2): [519, 417],
    (13, 3): [518, 381],
    (13, 4): [517, 346],
    (13, 5): [516, 311],
    (13, 6): [515, 277],
    (13, 7): [514, 243],
    (13, 8): [513, 209],
    (13, 9): [511, 176],
    (13, 10): [510, 144],
    (13, 11): [509, 112],
    (13, 12): [507, 80],
    (13, 13): [506, 49],
    (14, 1): [554, 452],
    (14, 2): [553, 417],
    (14, 3): [552, 381],
    (14, 4): [551, 346],
    (14, 5): [550, 311],
    (14, 6): [549, 277],
    (14, 7): [547, 243],
    (14, 8): [546, 210],
    (14, 9): [544, 177],
    (14, 10): [542, 145],
    (14, 11): [541, 113],
    (14, 12): [539, 81],
    (14, 13): [536, 49]
}


class BotServer:
    feedback = botFeedback()
    result = botResult()

    def __init__(self, bot_id):
        # Initialize node
        self.id = bot_id
        self.action_name = 'grid_robot_{}'.format(bot_id)
        rospy.init_node(self.action_name)
        self.rate = rospy.Rate(10)

        # Initialize action server node
        self.server = actionlib.SimpleActionServer(self.action_name, botAction,
                                                   self.execute,
                                                   auto_start=False)
        self.server.start()

        # Initialize cv objects
        self.cv_bridge = CvBridge()
        self.pose, self.image = None, None

        # Initialize pwm publisher
        self.msg = PwmCombined()
        self.pub = rospy.Publisher('/grid_robot_{}/pwm'.format(self.id),
                                   PwmCombined, queue_size=10)

        # Initialize PID parameters
        kp, ki, kd = [1, 1, 1, 1], [0, 0, 0, 0], [0, 0, 0, 0]
        self.kp, self.ki, self.kd = kp[self.id-1], ki[self.id-1], kd[self.id-1]
        self.intg, self.lastError, self.max_intg = 0.0, 0.0, 0.5
        base_speed, thresh_dist, thresh_angle = [
            90, 90, 90, 90], [15, 15, 15, 15], [45, 45, 45, 45]
        self.base_speed = base_speed[self.id-1]
        self.thresh_dist = thresh_dist[self.id-1]
        self.thresh_angle = thresh_angle[self.id-1]
        # self.cell_size = 38
        self.cell_size = 36
        self.min_pwm = 90

        # Log server start
        rospy.loginfo(self.action_name + " server initialized")

    # Execute goal callback
    def execute(self, goal):
        sub = rospy.Subscriber('/grid_robot/poses', GridPoseArray,
                               self.callback)
        time.sleep(0.5)

        # Linear PID

        while not rospy.is_shutdown():
            try:
                # Get current pose & target pose
                x, y = self.pose.x, self.pose.y

                # Get target pose from goal
                #tx = int(self.cell_size * goal.x
                # + self.cell_size / 2 + 0.5*(12-goal.y))
                #ty = int(self.cell_size * (12-goal.y) + self.cell_size / 2)

                tx = cordinates[(goal.x, goal.y)][0]
                ty = cordinates[(goal.x, goal.y)][1]
                print(tx, ty)

                # Calculate distance & angle
                target_angle = math.degrees(math.atan2(ty - y, tx - x))
                robot_angle = math.degrees(self.pose.theta)
                distance = math.sqrt((tx - x)**2 + (ty - y)**2)

                # Plot target and robot
                if self.image is not None:
                    cv2.arrowedLine(self.image, (int(x), int(y)),
                                    (int(tx), int(ty)), (0, 0, 255), 2)
                    cv2.circle(self.image, (int(tx), int(ty)),
                               self.thresh_dist, (255, 0, 0), 2)
                    cv2.imshow("grid_robot_{}".format(self.id), self.image)
                    cv2.waitKey(1)

                # Calculate error
                error = target_angle - robot_angle

                # Yaw angle correction
                if error > 180:
                    error -= 360
                if error < -180:
                    error += 360

                # Pid controller
                balance = self.pid(error)

                # Goal completion condition
                if distance < self.thresh_dist:
                    self.msg.left = 0
                    self.msg.right = 0
                    self.pub.publish(self.msg)
                    break

                # Publish feedback
                # self.feedback.x, self.feedback.y = self.pose.x, self.pose.y
                # self.server.publish_feedback(self.feedback)

                self.msg.left = int(self.base_speed + balance)
                self.msg.right = int(self.base_speed - balance)
                self.pub.publish(self.msg)
                self.rate.sleep()
            except AttributeError:
                continue

        # Angular PID
        while not rospy.is_shutdown():
            try:
                robot_angle = math.degrees(self.pose.theta)
                target_angle = goal.phi                    # already in degrees

                error = target_angle - robot_angle
                print(error)

                # Yaw angle correction
                if error > 180:
                    error -= 360
                if error < -180:
                    error += 360

                print(robot_angle, target_angle, error)
                if -self.thresh_angle < error < self.thresh_angle:
                    break

                balance = self.pid(error)
                if balance < self.min_pwm and balance > 0:
                    balance = self.min_pwm
                if balance > -self.min_pwm and balance < 0:
                    balance = -self.min_pwm

                self.msg.left = int(balance)
                self.msg.right = int(-balance)
                self.pub.publish(self.msg)
                self.rate.sleep()
            except AttributeError:
                continue

        # Stop the robot
        self.msg.left = 0
        self.msg.right = 0
        self.pub.publish(self.msg)

        if goal.servo:
            self.msg.servo = 1
            self.pub.publish(self.msg)
            time.sleep(1)
            self.msg.servo = 0
            self.pub.publish(self.msg)
            # if self.base_speed < 100:
            #     self.base_speed += 2

        # Publish result
        self.result.x, self.result.y = int(self.pose.x), int(self.pose.y)
        self.server.set_succeeded(self.result)

        # Unregister from image feed after goal completion
        sub.unregister()
        cv2.destroyAllWindows()

    def callback(self, msg):
        self.pose = [pose for pose in msg.poses if pose.id == self.id]
        self.pose = self.pose[0] if len(self.pose) else None
        try:
            self.image = self.cv_bridge.imgmsg_to_cv2(msg.image,
                                                      desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)

    def pid(self, error):
        prop = error
        self.intg += error
        if self.intg >= self.max_intg:
            self.intg = self.max_intg
        if self.intg <= -self.max_intg:
            self.intg = -self.max_intg
        diff = error - self.lastError
        self.lastError = error
        return self.kp * prop + self.ki * self.intg + self.kd * diff


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('id', type=int, default=1,
                        help='robot id, default: 1')
    args = parser.parse_args()
    server = BotServer(args.id)
    assert 1 <= args.id <= 4

    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
