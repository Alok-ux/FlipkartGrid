#!/usr/bin/env python3

import sys
import rospy
import actionlib
from grid_phase2_controller.msg import botAction, botGoal

rospy.init_node('pub')
client = actionlib.SimpleActionClient(
    'grid_robot_{}'.format(sys.argv[1]), botAction)
client.wait_for_server()
pose_list = [(221, 360), (219, 318), (184, 321), (180, 285),
             (141, 284), (100, 283), (101, 286)]
for pose in pose_list:
    goal = botGoal(x=pose[0], y=pose[1])
    client.send_goal(goal)
    print(client.wait_for_result())
