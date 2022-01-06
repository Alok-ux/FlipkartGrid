#! /usr/bin/env python3

import cv2
import math
import rospy
import argparse
import actionlib
from cv_bridge import CvBridge, cvBridgeError
from grid_phase2_controller.msg import botAction, botFeedback, botResult
from camera_driver.msg import GridPoseArray
from grid_transmitter.msg import PwmCombined


class BotServer:
    feedback = botFeedback()
    result = botResult()

    def __init__(self, bot_id):
        # Initialize node
        self.id = bot_id
        action_name = 'bot_{}_server'.format(bot_id)
        rospy.init_node(action_name)
        self.rate = rospy.Rate(10)

        # Initialize action server node
        self.server = actionlib.SimpleActionServer(action_name, botAction, self.execute, auto_start=False)
        self.server.start()

        # Initialize pose subscriber
        self.cv_bridge = CvBridge()
        self.pose, self.image = None, None
        rospy.Subscriber('/bot_pose', GridPoseArray, self.callback)
        
        # Initialize pwm publisher
        self.msg = PwmCombined()
        self.pub = rospy.Publisher('/grid_robot/pwm_{}'.format(self.id), PwmCombined, queue_size=10)
        
        # Initialize PID parameters
        self.kp, self.ki, self.kd = 1, 0, 0
        self.intg, self.lastError, self.max_intg = 0.0, 0.0, 0.5
        self.base_speed = 100

        # Log server start
        rospy.loginfo(action_name + " server initialized")

    def execute(self, goal):
        for target in goal:
            while not rospy.is_shutdown():
                # Get current pose & target pose
                x, y = self.pose.x, self.pose.y

                # TODO: Get target pose from goal
                tx, ty = target.x, target.y

                # Calculate distance & angle
                target_angle = math.degrees(math.atan2(ty - y, tx - x))
                robot_angle = self.pose.theta
                distance = math.sqrt((tx - x)**2 + (ty - y)**2)
                
                # Publish feedback
                self.feedback.pose = self.pose
                self.server.publish_feedback(self.feedback)

                # Plot target and robot
                if self.image is not None:
                    cv2.arrowedLine(self.image, (int(x), int(y)), (int(tx), int(ty)), (0, 0, 255), 2)
                    cv2.imshow("Bot {}".format(self.id), self.image)
                    cv2.waitKey(1)

                # Calculate error
                error = target_angle - robot_angle

                # Yaw angle correction
                if error > 180:
                    error -= 360
                if error < -180:
                    error += 360

                # Turning condition
                if distance < self.thresh_dist:
                    if -20 < error < 20:
                        break
                    else:
                        base_speed = 0
                else:
                    base_speed = self.base_speed

                # Pid controller
                balance = self.pid(error)
                self.msg.left = int(base_speed + balance)
                self.msg.right = int(base_speed - balance)
                self.pub.publish(self.msg)
                self.rate.sleep()
        
        # Stop the robot
        self.msg.left = 0
        self.msg.right = 0
        self.pub.publish(self.msg)

        # Publish result
        self.result.pose = self.pose
        self.server.set_succeeded(self.result)

    def shutdown(self):
        rospy.loginfo("Shutting down bot server")
        self.server.set_succeeded(self.result)
        self.server.shutdown()

    def callback(self, msg):
        self.pose = [pose for pose in msg.poses if pose.id == self.id]
        try:
            self.image = self.cv_bridge(msg.image, "bgr8")
        except cvBridgeError as e:
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

    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
    
    