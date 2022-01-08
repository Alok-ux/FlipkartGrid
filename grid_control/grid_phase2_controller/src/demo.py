#!/usr/bin/env python3

import sys
import math
import rospy
import actionlib
from grid_phase2_controller.msg import botAction, botGoal

rospy.init_node('pub')
client = actionlib.SimpleActionClient(
    'grid_robot_{}'.format(sys.argv[1]), botAction)
client.wait_for_server()
# pose_list = [(221, 360), (219, 318), (184, 321), (180, 285),
#              (141, 284), (100, 283), (101, 286)]
pose_list = [(1, 1), (1, 2)]
for i in range(len(pose_list)-1):
    x_i, y_i = pose_list[i][0], pose_list[i][1]
    x_j, y_j = pose_list[i+1][0], pose_list[i+1][1]
    phi = math.atan2(y_j - y_i, x_j - x_i)
    goal = botGoal(x=x_i, y=y_i, phi=phi)
    client.send_goal(goal)
    print(client.wait_for_result())
