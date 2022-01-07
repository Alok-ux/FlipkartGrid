#! /usr/bin/env python3

import sys
import rospy
import rospkg
import actionlib
from grid_phase2_controller.msg import Goal, botAction, botGoal
from cbs import solve
import csv


# class BotClient:
#     def __init__(self):
#         self.goal = botGoal()

#     def send_goal(self, goal):
#         pass

#     def callback_active(self):
#         rospy.loginfo('New Goal Received')

#     def callback_feedback(self, feedback):
#         pass

#     def callback_done(self, status, result):
#         pass


class BotClient:
    def __init__(self):
        self.param = {}
        self.client1 = actionlib.SimpleActionClient('bot1', botAction)
        self.client2 = actionlib.SimpleActionClient('bot2', botAction)
        self.client3 = actionlib.SimpleActionClient('bot3', botAction)
        self.client4 = actionlib.SimpleActionClient('bot4', botAction)
        self.map = {'dimensions': [15, 14],
                    'obstacles': [(0, 0), (0, 1), (0, 2), (0, 3), (0, 5), (0, 6), (0, 7), (0, 8), (0, 10), (0, 11), (0, 12), (0, 13), (3, 2), (3, 3), (4, 2), (4, 3), (3, 6), (3, 7), (4, 6), (4, 7), (3, 10), (3, 11), (4, 10), (4, 11), (7, 2), (7, 3), (8, 2), (8, 3), (7, 6), (7, 7), (8, 6), (8, 7), (7, 10), (7, 11), (8, 10), (8, 11), (11, 2), (11, 3), (12, 2), (12, 3), (11, 6), (11, 7), (12, 6), (12, 7), (11, 10), (11, 11), (12, 10), (12, 11)]}

        self.dict = {'1': (0, 4),
                     '2': (0, 9),
                     'Mumbai': (4, 12),
                     'Delhi': (8, 12),
                     'Kolkata': (12, 12),
                     'Chennai': (4, 8),
                     'Bengaluru': (8, 8),
                     'Hyderabad': (12, 8),
                     'Pune': (4, 4),
                     'Ahemdabad': (8, 4),
                     'Jaipur': (12, 4)}

        rospack = rospkg.RosPack()
        path = rospack.get_path('grid_phase2_controller')
        induct_list = [[], []]
        with open(path + "/data/Sample Data - Sheet1.csv") as file:
            reader = csv.reader(file)
            for i, row in enumerate(reader):
                if i == 0:
                    continue
                induct_list[int(row[1]) - 1].append({row[0]: row[-1]})

        self.status1 = 0
        self.status2 = 0
        self.status3 = 0
        self.status4 = 0

        self.param = []

        self.rate = rospy.Rate(10)

        solution = solve(param)

        for agent in solution:
            goal = botGoal(order=[Goal(t=d['t'], x=d['x'], y=d['y'])
                           for d in solution[agent]])
            print(goal)
            self.client.send_goal(goal, active_cb=self.callback)

        self.client1.send_goal(goal1, active_cb=self.callback_active_1,
                               feedback_cb=self.callback_feedback_1, done_cb=self.callback_done_1)
        self.client2.send_goal(goal2, active_cb=self.callback_active_2,
                               feedback_cb=self.callback_feedback_2, done_cb=self.callback_done_2)

        while True:
            print('The bots are moving', self.status1,
                  self.status2, self.status3, self.status4)
            self.rate.sleep()

    def publish_goals(self):

        param = {'agents': [{'start': [0, 4], 'goal': [
            4, 4], 'name': 'agent1'}], 'map': self.map}
        solution = solve(param)
        print("param: {} \nsolution: {}".format(param, solution))
        self.status1 = 1
        return solution

    def callback_active_1(self):
        print("The bot1 is moving in the destined path")

    def callback_active_2(self):
        print("The bot2 is moving in destined path")

    def callback_active_3(self):
        print("The bot3 is moving in the destined path")

    def callback_active_4(self):
        print("The bot4 is moving in the destined path")

    def callback_feedback_1(self, feedback1):
        if self.status1 != 1 and self.status1 != 2:
            self.current_pose_1 = feedback1

    def callback_feedback_2(self, feedback2):
        if self.status2 != 1 and self.status2 != 2:
            self.current_pose_2 = feedback2

    def callback_feedback_3(self, feedback3):
        if self.status3 != 1 and self.status3 != 2:
            self.current_pose_3 = feedback3

    def callback_feedback_4(self, feedback4):
        if self.status4 != 1 and self.status4 != 2:
            self.current_pose_4 = feedback4

    def callback_done_1(self, state, result):
        self.status1 = 2
        self.client2.cancel_goal()
        #self.client1.send_goal(goal1)
        #self.client1.send_goal(goal1, active_cb=self.callback_active_1, feedback_cb=self.callback_feedback_1, done_cb=self.callback_done_1)
        param = {'agents': [{'start': [5, 3], 'goal': [
            0, 9], 'name': 'agent1'}], 'map': self.map}
        solution = solve(param)
        print("param: {} \nsolution: {}".format(param, solution))

        param = {'agents': [{'start': self.current_pose_1,
                             'goal': [5, 3], 'name': 'agent1'}], 'map': self.map}
        solution = solve(param)
        print("param: {} \nsolution: {}".format(param, solution))

    def callback_done_2(self, state, result):
        self.status2 = 2
        self.client1.cancel_goal()
        #self.client1.send_goal(goal1)
        #self.client1.send_goal(goal1, active_cb=self.callback_active_1, feedback_cb=self.callback_feedback_1, done_cb=self.callback_done_1)
        param = {'agents': [{'start': [3, 5], 'goal': [
            0, 4], 'name': 'agent1'}], 'map': self.map}
        solution = solve(param)
        print("param: {} \nsolution: {}".format(param, solution))
        #print("param: {} \nsolution: {}".format(param, solution))

        param = {'agents': [{'start': self.current_pose_2,
                             'goal': [3, 5], 'name': 'agent2'}], 'map': self.map}
        solution = solve(param)
        print("param: {} \nsolution: {}".format(param, solution))


if __name__ == '__main__':
    try:
        rospy.init_node('action_client_trial')
        client = BotClient()
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
