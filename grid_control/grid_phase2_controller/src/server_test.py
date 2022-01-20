#! /usr/bin/env python3

import yaml
import math
import rospy
import rospkg
import argparse
import actionlib

from grid_phase2_controller.msg import botAction, botGoal
from grid_phase2_controller.cfg import pidConfig

from visualizer import Visualizer


class TestClient:
    def __init__(self, server_name) -> None:
        rospy.init_node('server_test_client')
        self.client = actionlib.SimpleActionClient(server_name, botAction)
        self.viz = Visualizer()
        self.client.wait_for_server()

    def __call__(self, goal_array) -> None:
        if len(goal_array[0]) == 3:
            for x, y, phi in goal_array:
                goal = botGoal(x=x, y=y, phi=phi)
                self.client.send_goal(goal)
                self.client.wait_for_result()
                
        elif len(goal_array[0]) == 2:
            for i in range(len(goal_array)-1):
                (x_i, y_i), (x_j, y_j) = goal_array[i], goal_array[i+1]

                phi = math.degrees(math.atan2(y_i - y_j, x_j - x_i))
                goal = botGoal(x=x_i, y=y_i, phi=phi)
                self.client.send_goal(goal)
                self.client.wait_for_result()        

    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test server')
    parser.add_argument('server', nargs='?', type=str, default='1', help='Server name')
    args = parser.parse_args()

    try:
        test_client = TestClient('grid_robot_{}'.format(args.server))
        while not rospy.is_shutdown():
            test_client([(1, 1, 90), (1, 4, 0), (2, 4, -90), (2, 1, 180)])
    except rospy.ROSInterruptException:
        pass
