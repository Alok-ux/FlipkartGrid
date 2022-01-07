#! /usr/bin/env python3

import csv
import yaml
import time
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

    def preempt_goal(self):
        self.client.cancel_goal()

    def send_goal(self, path):
        bot_goal = botGoal(
            order=[Goal(t=pose['t'], x=pose['x'], y=pose['y']) for pose in path])
        self.client.send_goal(
            bot_goal, feedback_cb=self.callback_feedback, done_cb=self.done_cb)

    def callback_feedback(self, feedback):
        self.curr_pose = [feedback.pose.x, feedback.y]


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
    param = {'agents': [{'start': [0, 4], 'goal': [3, 5], 'name': 'agent0'},
                        {'start': [0, 9], 'goal': [5, 3], 'name': 'agent1'}],
             'map': {'dimensions': [15, 14], 'obstacles': [(0, 0), (0, 1), (0, 2), (0, 3), (0, 5), (0, 6), (0, 7), (0, 8), (0, 10), (0, 11), (0, 12), (0, 13), (3, 2), (3, 3), (4, 2), (4, 3), (3, 6), (3, 7), (4, 6), (4, 7), (3, 10), (3, 11), (4, 10), (4, 11), (7, 2), (7, 3), (8, 2), (8, 3), (7, 6), (7, 7), (8, 6), (8, 7), (7, 10), (7, 11), (8, 10), (8, 11), (11, 2), (11, 3), (12, 2), (12, 3), (11, 6), (11, 7), (12, 6), (12, 7), (11, 10), (11, 11), (12, 10), (12, 11)]}}
    solution, env = solve(param)
    print("param: {} \nsolution: {}".format(param, solution))
    print("cost", env.compute_solution_cost(solution))
