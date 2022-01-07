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
        
        self.dict = {'1': (0,4),
                     '2': (0,9),
                     'Mumbai':(4,12),
                     'Delhi':(8,12),
                     'Kolkata':(12,12),
                     'Chennai':(4,8),
                     'Bengaluru':(8,8),
                     'Hyderabad':(12,8),
                     'Pune':(4,4),
                     'Ahemdabad':(8,4),
                     'Jaipur':(12,4) }

        rospack = rospkg.RosPack()
        path = rospack.get_path('grid_phase2_controller')
        induct_list = [[], []]
        with open(path + "/data/Sample Data - Sheet1.csv") as file:
            reader = csv.reader(file)
            for i, row in enumerate(reader):
                if i == 0:
                    continue
                induct_list[int(row[1]) - 1].append([row[0],row[-1]])
        self.status1 = 0 #initial
        self.status2 = 0
        self.status3 = 0
        self.status4 = 0

        self.param = []
        self.goal=[]

        self.rate = rospy.Rate(10)

        #solution = solve(self.param)

        #for agent in solution:
            #goal = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
            #print(goal)
            #self.client.send_goal(goal, active_cb=self.callback)

        self.client1.send_goal(self.goal, active_cb=self.callback_active_1, feedback_cb=self.callback_feedback_1, done_cb=self.callback_done_1)
        self.client2.send_goal(self.goal, active_cb=self.callback_active_2, feedback_cb=self.callback_feedback_2, done_cb=self.callback_done_2)
        self.client3.send_goal(self.goal, active_cb=self.callback_active_3, feedback_cb=self.callback_feedback_3, done_cb=self.callback_done_3)
        self.client4.send_goal(self.goal, active_cb=self.callback_active_4, feedback_cb=self.callback_feedback_4, done_cb=self.callback_done_4)
       
        while True:
            print('The bots are moving', self.status1, self.status2, self.status3, self.status4)
            self.rate.sleep()


    def publish_goals(self):

        param = {'agents': [{'start': [0, 4], 'goal': [4, 4], 'name': 'agent1'}], 'map': self.map}
        solution= solve(param)
        print("param: {} \nsolution: {}".format(param, solution))
        self.status1 = 1
        #return solution

    def callback_active_1(self):
        print("The bot1 is moving in the destined path")
        self.status1 = 1
    
    def callback_active_2(self):
        print("The bot2 is moving in destined path")
        self.status2 = 1


    def callback_active_3(self):
        print("The bot3 is moving in the destined path")
        self.status3 = 1


    def callback_active_4(self):
        print("The bot4 is moving in the destined path") 
        self.status4 = 1



    def callback_feedback_1(self, feedback1):
        if  self.status1 !=2:
            self.current_pose_1 = feedback1

        if feedback1 == [0,4]:
            self.param = {'agent': [{'start': [0,4], 'goal': [12,12], 'name': 'agent1'}], 'map': self.map}#here goal should be next destination and start is the induct station
            solution= solve(self.param)
            print("param: {} \nsolution: {}".format(self.param, solution))
            for agent in solution:
                goal1= botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
                print(goal1)
                self.client1.send_goal(goal1,active_cb=self.callback_active_1, feedback_cb=self.callback_feedback_1, done_cb=self.callback_done_1)
                self.status1 = 3
                print("Now the bot1 is returning to the induct station")

       

    def callback_feedback_2(self, feedback2):
        if self.status2 !=2:
            self.current_pose_2 = feedback2

        if feedback2 == [0,9]:
            self.param = {'agent': [{'start': [0,9], 'goal': [8,12], 'name': 'agent2'}], 'map': self.map}#here goal should be next destination and start is the induct station
            solution= solve(self.param)
            print("param: {} \nsolution: {}".format(self.param, solution))
            for agent in solution:
                goal2 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
                print(goal2)
                self.client2.send_goal(goal2,active_cb=self.callback_active_2, feedback_cb=self.callback_feedback_2, done_cb=self.callback_done_2)
                self.status2 = 3
                print("Now the bot2 is returning to the induct station")


    def callback_feedback_3(self, feedback3):
        if self.status3 !=2 :
            self.current_pose_3 = feedback3

        if feedback3 == [0,4]:
            self.param = {'agent': [{'start': [0,4], 'goal': [4,12], 'name': 'agent3'}], 'map': self.map}#here goal should be next destination and start is the induct station
            solution= solve(self.param)
            print("param: {} \nsolution: {}".format(self.param, solution))
            for agent in solution:
                goal3 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
                print(goal3)
                self.client3.send_goal(goal3, active_cb=self.callback_active_3, feedback_cb=self.callback_feedback_3, done_cb=self.callback_done_3)
                self.status3 = 3
                print("Now the bot3 is returning to the induct station")


    def callback_feedback_4(self, feedback4):
        if self.status4 !=2 :
            self.current_pose_4 = feedback4

        if feedback4 == [0,9]:
            self.param = {'agent': [{'start': [0,9], 'goal': [4,4], 'name': 'agent4'}], 'map': self.map}#here goal should be next destination and start is the induct station
            solution= solve(self.param)
            print("param: {} \nsolution: {}".format(self.param, solution))
            for agent in solution:
                goal4 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
                print(goal4)
                self.client4.send_goal(goal4, active_cb=self.callback_active_4, feedback_cb=self.callback_feedback_4, done_cb=self.callback_done_4)
                self.status4 = 3
                print("Now the bot4 is returning to the induct station")


    def callback_done_1(self, state, result):
        self.status1 = 2
        print("bot1 reached the destination , now returning to the induct station")
        self.client2.cancel_goal()
        self.client3.cancel_goal()
        self.client4.cancel_goal()
        #self.client1.send_goal(goal1)
        #self.client1.send_goal(goal1, active_cb=self.callback_active_1, feedback_cb=self.callback_feedback_1, done_cb=self.callback_done_1)

        self.param = {'agent': [{'start': [4,4], 'goal': [0, 4], 'name': 'agent1'}], 'map': self.map}#here goal should be the induct station and start is the destination
        solution= solve(self.param)
        print("param: {} \nsolution: {}".format(self.param, solution))
        for agent in solution:
            goal1 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
            print(goal1)
            self.client1.send_goal(goal1, active_cb=self.callback_active_1, feedback_cb=self.callback_feedback_1, done_cb=self.callback_done_1)
            self.status1 = 3
            print("Now the bot1 is returning to the induct station")

        if self.status2 !=2 and self.status2 != 3 :

            self.param = {'agent': [{'start': self.current_pose_2, 'goal': [4, 12], 'name': 'agent2'}], 'map': self.map}#here goal should be the destination and start is the current_pose
            solution= solve(self.param)
            print("param: {} \nsolution: {}".format(self.param, solution))
            for agent in solution:
                goal2 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
                print(goal2)
                self.client2.send_goal(goal2,active_cb=self.callback_active_2, feedback_cb=self.callback_feedback_2, done_cb=self.callback_done_2)
                print("Bot2 is on the way to destination")

        if self.status3 !=2 and self.status3 != 3 :

            self.param = {'agent': [{'start': self.current_pose_3, 'goal': [8, 12], 'name': 'agent3'}], 'map': self.map}#here goal should be the destination and start is the current_pose
            solution= solve(self.param)
            print("param: {} \nsolution: {}".format(self.param, solution))
            for agent in solution:
                goal3 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
                print(goal3)
                self.client3.send_goal(goal3,active_cb=self.callback_active_3, feedback_cb=self.callback_feedback_3, done_cb=self.callback_done_3)
                print("Bot3 is on the way to destination") 

        if self.status4 !=2 and self.status4 != 3 :

            self.param = {'agent': [{'start': self.current_pose_4, 'goal': [12, 12], 'name': 'agent3'}], 'map': self.map}#here goal should be the destination and start is the current_pose
            solution= solve(self.param)
            print("param: {} \nsolution: {}".format(self.param, solution))
            for agent in solution:
                goal4 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
                print(goal4)
                self.client4.send_goal(goal4,active_cb=self.callback_active_4, feedback_cb=self.callback_feedback_4, done_cb=self.callback_done_4)
                print("Bot4 is on the way to destination") 
        #param = {'agents': [{'start': self.current_pose_1, 'goal': [5,3], 'name': 'agent1'}], 'map': self.map}
        #solution= solve(param)
        #print("param: {} \nsolution: {}".format(param, solution))


    def callback_done_2(self, state, result):
        self.status2 = 2
        print("bot2 reached the destination , now returning to the induct station")
        self.client1.cancel_goal()
        self.client3.cancel_goal()
        self.client4.cancel_goal()
        #self.client1.send_goal(goal1)
        #self.client1.send_goal(goal1, active_cb=self.callback_active_1, feedback_cb=self.callback_feedback_1, done_cb=self.callback_done_1)

        self.param = {'agent': [{'start': [4,12], 'goal': [0, 9], 'name': 'agent2'}], 'map': self.map}#here goal should be the induct station and start is the destination
        solution= solve(self.param)
        print("param: {} \nsolution: {}".format(self.param, solution))
        for agent in solution:
            goal2 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
            print(goal2)
            self.client2.send_goal(goal2, active_cb=self.callback_active_2, feedback_cb=self.callback_feedback_2, done_cb=self.callback_done_2)
            self.status2 = 3
            print("Now the bot2 is returning to the induct station")

        if self.status1 !=2 and self.status1 != 3 :

            self.param = {'agent': [{'start': self.current_pose_1, 'goal': [4, 4], 'name': 'agent1'}], 'map': self.map}#here goal should be the destination and start is the current_pose
            solution= solve(self.param)
            print("param: {} \nsolution: {}".format(self.param, solution))
            for agent in solution:
                goal1 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
                print(goal1)
                self.client1.send_goal(goal1, active_cb=self.callback_active_1, feedback_cb=self.callback_feedback_1, done_cb=self.callback_done_1)
                print("Bot1 is on the way to destination")

        if self.status3 !=2 and self.status3 != 3 :

            self.param = {'agent': [{'start': self.current_pose_3, 'goal': [8, 12], 'name': 'agent3'}], 'map': self.map}#here goal should be the destination and start is the current_pose
            solution= solve(self.param)
            print("param: {} \nsolution: {}".format(self.param, solution))
            for agent in solution:
                goal3 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
                print(goal3)
                self.client3.send_goal(goal3, active_cb=self.callback_active_3, feedback_cb=self.callback_feedback_3, done_cb=self.callback_done_3)
                print("Bot3 is on the way to destination") 

        if self.status4 !=2 and self.status4 != 3 :

            self.param = {'agent': [{'start': self.current_pose_4, 'goal': [12, 12], 'name': 'agent4'}], 'map': self.map}#here goal should be the destination and start is the current_pose
            solution= solve(self.param)
            print("param: {} \nsolution: {}".format(self.param, solution))
            for agent in solution:
                goal4 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
                print(goal4)
                self.client4.send_goal(goal4, active_cb=self.callback_active_4, feedback_cb=self.callback_feedback_4, done_cb=self.callback_done_4)
                print("Bot4 is on the way to destination")


    def callback_done_3(self, state, result):
        self.status3 = 2
        print("bot3 reached the destination , now returning to the induct station")
        self.client2.cancel_goal()
        self.client1.cancel_goal()
        self.client4.cancel_goal()
        #self.client1.send_goal(goal1)
        #self.client1.send_goal(goal1, active_cb=self.callback_active_1, feedback_cb=self.callback_feedback_1, done_cb=self.callback_done_1)

        self.param = {'agent': [{'start': [8,12], 'goal': [0, 4], 'name': 'agent1'}], 'map': self.map}#here goal should be the induct station and start is the destination
        solution= solve(self.param)
        print("param: {} \nsolution: {}".format(self.param, solution))
        for agent in solution:
            goal3 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
            print(goal3)
            self.client3.send_goal(goal3, active_cb=self.callback_active_3, feedback_cb=self.callback_feedback_3, done_cb=self.callback_done_3)
            self.status3 = 3
            print("Now the bot3 is returning to the induct station")

        if self.status2 !=2 and self.status2 != 3 :

            self.param = {'agent': [{'start': self.current_pose_2, 'goal': [4, 12], 'name': 'agent2'}], 'map': self.map}#here goal should be the destination and start is the current_pose
            solution= solve(self.param)
            print("param: {} \nsolution: {}".format(self.param, solution))
            for agent in solution:
                goal2 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
                print(goal2)
                self.client2.send_goal(goal2, active_cb=self.callback_active_2, feedback_cb=self.callback_feedback_2, done_cb=self.callback_done_2)
                print("Bot2 is on the way to destination")

        if self.status1 !=2 and self.status1 != 3 :

            self.param = {'agent': [{'start': self.current_pose_1, 'goal': [4, 4], 'name': 'agent1'}], 'map': self.map}#here goal should be the destination and start is the current_pose
            solution= solve(self.param)
            print("param: {} \nsolution: {}".format(self.param, solution))
            for agent in solution:
                goal1 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
                print(goal1)
                self.client1.send_goal(goal1, active_cb=self.callback_active_1, feedback_cb=self.callback_feedback_1, done_cb=self.callback_done_1)
                print("Bot1 is on the way to destination") 

        if self.status4 !=2 and self.status4 != 3 :

            self.param = {'agent': [{'start': self.current_pose_4, 'goal': [12, 12], 'name': 'agent3'}], 'map': self.map}#here goal should be the destination and start is the current_pose
            solution= solve(self.param)
            print("param: {} \nsolution: {}".format(self.param, solution))
            for agent in solution:
                goal4 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
                print(goal4)
                self.client4.send_goal(goal4, active_cb=self.callback_active_4, feedback_cb=self.callback_feedback_4, done_cb=self.callback_done_4)
                print("Bot4 is on the way to destination")


    def callback_done_4(self, state, result):
        self.status4 = 2
        print("bot4 reached the destination , now returning to the induct station")
        self.client2.cancel_goal()
        self.client3.cancel_goal()
        self.client1.cancel_goal()
        #self.client1.send_goal(goal1)
        #self.client1.send_goal(goal1, active_cb=self.callback_active_1, feedback_cb=self.callback_feedback_1, done_cb=self.callback_done_1)

        self.param = {'agent': [{'start': [0,9], 'goal': [12,12], 'name': 'agent4'}], 'map': self.map}#here goal should be the induct station and start is the destination
        solution= solve(self.param)
        print("param: {} \nsolution: {}".format(self.param, solution))
        for agent in solution:
            goal4 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
            print(goal4)
            self.client4.send_goal(goal4, active_cb=self.callback_active_4, feedback_cb=self.callback_feedback_4, done_cb=self.callback_done_4)
            self.status4 = 3
            print("Now the bot4 is returning to the induct station")

        if self.status2 !=2 and self.status2 != 3 :

            self.param = {'agent': [{'start': self.current_pose_2, 'goal': [4, 12], 'name': 'agent2'}], 'map': self.map}#here goal should be the destination and start is the current_pose
            solution= solve(self.param)
            print("param: {} \nsolution: {}".format(self.param, solution))
            for agent in solution:
                goal2 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
                print(goal2)
                self.client2.send_goal(goal2, active_cb=self.callback_active_2, feedback_cb=self.callback_feedback_2, done_cb=self.callback_done_2)
                print("Bot2 is on the way to destination")

        if self.status3 !=2 and self.status3 != 3 :

            self.param = {'agent': [{'start': self.current_pose_3, 'goal': [8, 12], 'name': 'agent3'}], 'map': self.map}#here goal should be the destination and start is the current_pose
            solution= solve(self.param)
            print("param: {} \nsolution: {}".format(self.param, solution))
            for agent in solution:
                goal3 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
                print(goal3)
                self.client3.send_goal(goal3, active_cb=self.callback_active_3, feedback_cb=self.callback_feedback_3, done_cb=self.callback_done_3)
                print("Bot3 is on the way to destination") 

        if self.status1 !=2 and self.status1 != 3 :

            self.param = {'agent': [{'start': self.current_pose_1, 'goal': [4 4], 'name': 'agent1'}], 'map': self.map}#here goal should be the destination and start is the current_pose
            solution= solve(self.param)
            print("param: {} \nsolution: {}".format(self.param, solution))
            for agent in solution:
                goal1 = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
                print(goal1)
                self.client1.send_goal(goal1, active_cb=self.callback_active_1, feedback_cb=self.callback_feedback_1, done_cb=self.callback_done_1)
                print("Bot1 is on the way to destination")

       

if __name__ == '__main__':
    try:
        rospy.init_node('action_client_trial')
        client = BotClient()
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")

        
        












