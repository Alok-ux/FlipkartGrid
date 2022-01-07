#!/usr/bin/env python3

import sys
import time
import rospy
import actionlib
from grid_phase2_controller.msg import botAction, botGoal, Goal


rospy.init_node('pub')
client = actionlib.SimpleActionClient(
    'grid_robot_{}'.format(sys.argv[1]), botAction)
client.wait_for_server()
pose_list = [(221, 360), (219, 318), (184, 321), (180, 285),
             (141, 284), (100, 283), (101, 286)]
goal = botGoal(order=[Goal(t=0, x=p[0], y=p[1]) for p in pose_list])
client.send_goal(goal)
print(client.wait_for_result())
