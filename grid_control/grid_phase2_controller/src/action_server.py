#! /usr/bin/env python

import rospy
import actionlib
from grid_phase2_controller.msg import botAction, botFeedback, botResult

class BotServer:
    feedback = botFeedback()
    result = botResult()

    def __init__(self):
        pass
