#! /usr/bin/env python3

import rospy
import actionlib
from grid_phase2_controller.msg import botAction, botFeedback, botResult

class BotServer:
    feedback = botFeedback()
    result = botResult()

    def __init__(self):
        self.server = actionlib.SimpleActionServer('bot_action', botAction, self.execute, auto_start=False)
        self.server.start()
        self.rate = rospy.Rate(10)

    def execute(self, goal):
        self.feedback.dist = 0
        self.feedback.ang = 0
        self.result.dist = 0
        self.result.ang = 0
        self.server.publish_feedback(self.feedback)
        x1 = goal.x1
        y1 = goal.y1
        x2 = goal.x2
        y2 = goal.y2
        while not rospy.is_shutdown():
            if abs(x1-x2) < 5 and abs(y1-y2) < 5:
                self.result.dist = 0
                self.result.ang = 0
                self.server.set_succeeded(self.result)
                break
            else:
                x1 = x2
                y1 = y2
            self.feedback.dist = abs(x1-x2)
            self.feedback.ang = abs(y1-y2)
            self.server.publish_feedback(self.feedback)
            self.rate.sleep()

    def shutdown(self):
        rospy.loginfo("Shutting down bot server")
        self.server.set_succeeded(self.result)
        self.server.shutdown()

    def callback(self, msg):
        self.pose = msg.poses


    