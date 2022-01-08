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
        self.kp, self.ki, self.kd = 1, 0, 0
        self.intg, self.lastError, self.max_intg = 0.0, 0.0, 0.5
        self.base_speed = 100
        self.thresh_dist = 15

        # Log server start
        rospy.loginfo(self.action_name + " server initialized")

    def execute(self, goal):
        success = True
        sub = rospy.Subscriber('/grid_robot/poses', GridPoseArray,
                               self.callback)
        time.sleep(0.5)
        while not rospy.is_shutdown():
            try:
                # Get current pose & target pose
                x, y = self.pose.x, self.pose.y

                # TODO: Get target pose from goal
                tx, ty = goal.x, goal.y

                # Calculate distance & angle
                target_angle = math.degrees(math.atan2(ty - y, tx - x))
                robot_angle = math.degrees(self.pose.theta)
                distance = math.sqrt((tx - x)**2 + (ty - y)**2)

                # Publish feedback
                self.feedback.x, self.feedback.y = self.pose.x, self.pose.y
                self.server.publish_feedback(self.feedback)

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
                    break

                # Orient first if angle is too high
                # TODO: Write control code

                if -20 < error < 20:
                    base_speed = self.base_speed
                else:
                    base_speed = 0
                    self.orient(error)

                    # if -100 < balance < 100:
                    #     if balance > 0:
                    #         balance += 50
                    #     else:
                    #         balance -= 50

                self.msg.left = int(base_speed + balance)
                self.msg.right = int(base_speed - balance)
                self.pub.publish(self.msg)
                self.rate.sleep()

                # Preempt signal
                if self.server.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self.action_name)
                    self.server.set_preempted()
                    success = False
                    break
            except AttributeError:
                continue

        # Stop the robot
        self.msg.left = 0
        self.msg.right = 0
        self.pub.publish(self.msg)

        # Publish result
        if success:
            self.result.x, self.result.y = self.pose.x, self.pose.y
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

    def orient(self, error):
        print("Orient")
        if error > 20 or error < -20:
            balance = self.pid(error)
            self.msg.left = int(balance)
            self.msg.right = int(-balance)
            print(self.msg)
            self.pub.publish(self.msg)


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
