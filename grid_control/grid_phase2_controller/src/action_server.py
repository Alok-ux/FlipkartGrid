#! /usr/bin/env python3

import cv2
import csv
import json
import time
import math
import rospy
import rospkg
import argparse
import actionlib

from cv_bridge import CvBridge, CvBridgeError
from grid_phase2_controller.msg import botAction, botFeedback, botResult
from camera_driver.msg import GridPoseArray
from grid_transmitter.msg import PwmCombined

from grid_phase2_controller.cfg import pidConfig
from dynamic_reconfigure.server import Server


class BotServer:
    feedback = botFeedback()
    result = botResult()

    def __init__(self, bot_id, csv_path, params_path):
        # Initialize node
        self.id = bot_id
        self.action_name = 'grid_robot_{}'.format(bot_id)
        rospy.init_node(self.action_name)
        self.rate = rospy.Rate(10)

        self.params_path = params_path
        self.load()

        Server(pidConfig, self.dyn_callback)

        # Initialize action server node
        self.server = actionlib.SimpleActionServer(self.action_name, botAction,
                                                   self.execute,
                                                   auto_start=False)
        self.server.start()

        # Initialize cv objects
        self.cv_bridge = CvBridge()
        self.pose, self.image = None, None

        # Initialize coordinates
        with open(csv_path, 'r') as file:
            reader = csv.reader(file)
            self.coordinates = {(int(row[0]), int(row[1])): (
                int(row[2]), int(row[3])) for row in reader}

        # Initialize pwm publisher
        self.msg = PwmCombined()
        self.pub = rospy.Publisher('/grid_robot_{}/pwm'.format(self.id),
                                   PwmCombined, queue_size=10)

        # Initialize PID parameters
        self.intg, self.lastError, self.max_intg = 0.0, 0.0, 0.5
        # self.cell_size = 38
        self.cell_size = 36

        # Log server start
        rospy.loginfo(self.action_name + " server initialized")

    def save(self):
        with open(self.params_path, 'r') as file:
            data = json.load(file)
            data[self.id - 1].update(self.params)
        with open(self.params_path, 'w') as file:
            json.dump(data, file)

    def load(self):
        with open(self.params_path, 'r') as file:
            self.params = json.load(file)[self.id - 1]

    def dyn_callback(self, config, level):
        if config['load']:
            self.load()
            config.update(self.params)
            config['load'] = False

        if config['save']:
            self.save()
            config['save'] = False

        self.params.update(
            {key: config[key] for key in config if key not in ['groups', 'save', 'load']})
        return config

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
                # tx = int(self.cell_size * goal.x + self.cell_size / 2 + 0.5*(12-goal.y))
                # ty = int(self.cell_size * (12-goal.y) + self.cell_size / 2)

                tx, ty = self.coordinates[(goal.x, goal.y)]
                # ty = coordinates[(goal.x, (12-goal.y))][1]

                # Calculate distance & angle
                target_angle = math.degrees(math.atan2(ty - y, tx - x))
                robot_angle = math.degrees(self.pose.theta)
                distance = math.sqrt((tx - x)**2 + (ty - y)**2)

                # Plot target and robot
                if self.image is not None:
                    cv2.arrowedLine(self.image, (int(x), int(y)),
                                    (int(tx), int(ty)), (0, 0, 255), 2)
                    cv2.circle(self.image, (int(tx), int(ty)),
                               int(self.params['thresh_dist']), (255, 0, 0), 2)
                    # cv2.imshow("grid_robot_{}".format(self.id), self.image)
                    # cv2.waitKey(1)

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
                if distance < self.params['thresh_dist']:
                    self.msg.left = 0
                    self.msg.right = 0
                    self.pub.publish(self.msg)
                    break

                # Publish feedback
                # self.feedback.x, self.feedback.y = self.pose.x, self.pose.y
                # self.server.publish_feedback(self.feedback)

                self.msg.left = int(self.params['base_speed'] + balance)
                self.msg.right = int(self.params['base_speed'] - balance)
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
                if -self.params['thresh_angle'] < error < self.params['thresh_angle']:
                    break

                balance = self.pid(error)
                if balance < self.params['min_pwm'] and balance > 0:
                    balance = self.params['min_pwm']
                if balance > -self.params['min_pwm'] and balance < 0:
                    balance = -self.params['min_pwm']

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
        return self.params['kp'] * prop + self.params['ki'] * self.intg + self.params['kd'] * diff


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('id', type=int, default=1,
                        help='robot id, default: 1')
    parser.add_argument('-c', '--coordinates', type=str, default='coordinates.csv',
                        help='relative path to coordinates.csv file')
    parser.add_argument('-p', '--params', type=str, default='params.json',
                        help='relative path to params.csv file')
    args = parser.parse_args()

    path = rospkg.RosPack().get_path('grid_phase2_controller') + '/data/'
    server = BotServer(args.id, path + args.coordinates, path + args.params)
    assert 1 <= args.id <= 4

    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
