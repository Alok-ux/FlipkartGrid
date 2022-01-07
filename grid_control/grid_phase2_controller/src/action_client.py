#! /usr/bin/env python3

import csv
import yaml
import time
import rospy
import rospkg
import actionlib

from grid_phase2_controller.msg import botAction, botGoal, Goal
from cbs import solve


class InductStation:
    def __init__(self, id):
        self.id = id
        rospack = rospkg.RosPack()
        self.path = rospack.get_path('grid_phase2_controller') + \
            "/data/Sample Data - Sheet1.csv"
        self.induct_list = []
        self.read_csv()

    def read_csv(self):
        with open(self.path) as file:
            reader = csv.reader(file)
            for i, row in enumerate(reader):
                if i == 0:
                    continue
                if int(row[1]) == self.id:
                    self.induct_list.append({row[0]: row[-1]})

    def __len__(self):
        return len(self.induct_list)

    def __iter__(self):
        return iter(self.induct_list)

    def pop(self):
        return self.induct_list.pop(0)


class Robot:
    def __init__(self, id, done_cb):
        self.client = actionlib.SimpleActionClient(
            'grid_robot_{}'.format(id), botAction)
        self.done_cb = done_cb
        self.client.wait_for_server()
        self.curr_pose, self.prev_goal = None, None
        self.loaded_parcel = list()

    def preempt_goal(self):
        self.client.cancel_goal()

    def send_goal(self, goal):
        bot_goal = botGoal(x=goal[0], y=goal[1])
        self.client.send_goal(
            bot_goal, feedback_cb=self.callback_feedback, done_cb=self.done_cb)

    def callback_feedback(self, feedback):
        self.curr_pose = [feedback.x, feedback.y]


class Automata:
    def __init__(self, num_bots, num_induct_stations, yaml_path):
        self.induct_station = [InductStation(i)
                               for i in range(num_induct_stations)]
        self.bots = [Robot(i, self.done_cb) for i in range(num_bots)]
        self.drop_location_pose = {'x': 0, 'y': 0}

        with open(yaml_path, 'r') as param_file:
            try:
                self.param = yaml.load(param_file, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

    def execute(self, num_pkg=-1):
        pass

    def new_goal(self, station_id):
        return self.induct_station[station_id].pop()

    def done_cb(self, status, result):
        for i, bot in enumerate(self.bots):
            if result.result.id == bot.id:
                # Goto induct station
                self.param['agents'][i]['start'] = bot.curr_pose
                self.param['agents'][i]['goal'] = self.induct_station[i//4].pose
                self.param['agents'][i]['name'] = 'agent'+str(i)

                # Already in induct station & goto new destination
                for i in self.induct_station:
                    if i.pose == bot.curr_pose:
                        bot.prev_goal = self.param['agents'][i]['goal'] = self.induct_station[i].pop(
                        )
                        time.sleep(2)
                        break
                continue

            bot.preempt_goal()
            self.param['agents'][i]['start'] = bot.curr_pose
            self.param['agents'][i]['goal'] = bot.prev_goal
            self.param['agents'][i]['name'] = 'agent'+str(i)

        # Get path from CBS algorithm
        solution, _ = solve(self.param)

        for i, bot in enumerate(self.bots):
            bot.send_goal(solution['agent'+str(i)])


if __name__ == '__main__':
    try:
        automata = Automata()
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
