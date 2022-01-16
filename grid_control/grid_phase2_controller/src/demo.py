#!/usr/bin/env python3

import sys
import math
import rospy
import actionlib
from grid_phase2_controller.msg import botAction, botGoal

rospy.init_node('pub')
client1 = actionlib.SimpleActionClient(
    'grid_robot_2', botAction)
client2 = actionlib.SimpleActionClient(
    'grid_robot_3', botAction)

client1.wait_for_server()
client2.wait_for_server()

# pose_list = [(221, 360), (219, 318), (184, 321), (180, 285),
#              (141, 284), (100, 283), (101, 286)]
pose_list1 = [(0, 4), (1, 4), (1, 5), (2, 5), (3, 5),
              (4, 5), (5, 5), (5, 4), (6, 4), (7, 4)]
pose_list2 = [(0, 9), (1, 9), (1, 8), (2, 8),
              (3, 8), (4, 8), (5, 8), (5, 7), (5, 6), (4, 6)]
pose_list1.append((7, 5))
for i in range(1, 10):
    if i == len(pose_list1)-1:
        pose_list1.append((5, 9))
        x_i, y_i = pose_list1[i][0], pose_list1[i][1]
        x_j, y_j = pose_list1[i+1][0], pose_list1[i+1][1]
        phi = math.degrees(math.atan2(y_i - y_j, x_j - x_i))
        goal1 = botGoal(x=x_i, y=y_i, phi=phi)

    else:

        x_i, y_i = pose_list1[i][0], pose_list1[i][1]
        x_j, y_j = pose_list1[i+1][0], pose_list1[i+1][1]
        phi = math.degrees(math.atan2(y_i - y_j, x_j - x_i))
        goal1 = botGoal(x=x_i, y=y_i, phi=phi)

    x_i, y_i = pose_list2[i][0], pose_list2[i][1]
    x_j, y_j = pose_list2[i+1][0], pose_list2[i+1][1]
    phi = math.degrees(math.atan2(y_i - y_j, x_j - x_i))
    goal2 = botGoal(x=x_i, y=y_i, phi=phi)

    client1.send_goal(goal1)
    client2.send_goal(goal2)
    print(client1.wait_for_result())
    print(client2.wait_for_result())
