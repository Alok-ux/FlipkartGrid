#! /usr/bin/env python3

import csv
import yaml
import math
import rospy
import rospkg
import actionlib

from grid_phase2_controller.msg import botAction, botGoal
from cbs import solve


class InductStation:
    def __init__(self, id: int, pose: tuple, path: str) -> None:
        '''
        Constructor is called when InductStation object is created
        params: id (int): id of the induct station
                pose (tuple): tuple consisting position of the induct station
                path (str): path of the package csv file
        return: None
        '''
        assert id != 0          # asserts id not equal to 0 as id starts from 1
        self.id = id            # station id e.q. IS1, IS2
        self.pose = pose        # position of induct station e.q. (1, 2)

        self.__induct_list = []   # packages available at the induct station
        self.__read_csv(path)     # read package data from csv file

        self.__drop_pose = {'Mumbai': [(3, 9), (3, 10)],
                            'Delhi': [(7, 9), (7, 10)],
                            'Kolkata': [(11, 9), (11, 10)],
                            'Chennai': [(3, 5), (3, 6)],
                            'Bengaluru': [(7, 5), (7, 6)],
                            'Hyderabad': [(11, 5), (11, 6)],
                            'Pune': [(3, 1), (3, 2)],
                            'Ahmedabad': [(7, 1), (7, 2)],
                            'Jaipur': [(11, 1), (11, 2)]}   # position of drop

    def __read_csv(self, path: str) -> None:
        '''
        reads csv file containing package list and corresponding drop location
        params: path (str): path of the csv file
        return: None
        '''
        with open(path) as file:
            reader = csv.reader(file)
            for i, row in enumerate(reader):
                if i == 0:
                    continue
                if int(row[1]) == self.id:
                    self.__induct_list.append(row[-1])

    def __len__(self) -> int:
        '''
        This function is called when len(obj) is executed
        params: None
        return: (int): length of the induct list
        '''
        return len(self.__induct_list)

    def __iter__(self) -> iter:
        '''
        This function is called when iter(obj) is executed
        params: None
        return: (iter): iterator of induct list
        '''
        return iter(self.__induct_list)

    def __getitem__(self, key: str) -> list:
        '''
        This funtion is called when obj[key] is executed
        params: key (str): drop location name
        return: list (list): drop location position
        '''
        return self.__drop_pose[key]

    def pop(self) -> int:
        '''
        This function pops an entry from the beginning of the induct list
        params: None
        return: (int): poped entry from the beginning
        '''
        return self.__induct_list.pop(0)


class Robot:
    def __init__(self, id):
        self.client = actionlib.SimpleActionClient(
            'grid_robot_{}'.format(id), botAction)
        self.client.wait_for_server()

    def send_goal(self, goal_i, goal_j=None, phi=0, servo=0):
        if goal_j:
            phi = math.degrees(math.atan2(goal_i['y'] - goal_j['y'],
                                          goal_j['x'] - goal_i['x']))

        bot_goal = botGoal(x=goal_i['x'], y=goal_i['y'], phi=phi, servo=servo)
        # return 'x: {}, y: {}, phi: {}, servo: {}'.format(goal_i['x'], goal_i['y'], phi, servo)
        self.client.send_goal(bot_goal)


class Automata:
    def __init__(self, num_bots, induct_station, data_path, map_path):
        self.induct_station = [InductStation(i+1, induct_station[i], data_path)
                               for i in range(len(induct_station))]
        # i + 1 is to start bot 2 & 3 in place of 0 & 1
        self.bots = [Robot(i+2) for i in range(num_bots)]

        with open(map_path, 'r') as param_file:
            try:
                self.param = yaml.load(param_file, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

    def execute(self, num_pkg=-1):
        self.induct_station[0].pop()
        self.induct_station[1].pop()
        self.param['agents'] = list()

        goal_pose = list()
        for i in range(len(self.bots)):
            goal_name = self.induct_station[i].pop()
            goal_pose.append(self.induct_station[i][goal_name])

            self.param['agents'].append({'start': self.induct_station[i].pose,
                                         'goal': goal_pose[i][0],
                                         'name': 'agent'+str(i)
                                         })
        print('param', self.param)
        print('goal_pose', goal_pose)
        solution, _ = solve(self.param)
        print('solution', solution)

        # Find min len array
        sol_len = []
        for i, agent in enumerate(solution):
            x, y = goal_pose[i][1][0], goal_pose[i][1][1]
            solution[agent].append({'t': len(solution[agent]), 'x': x, 'y': y})
            sol_len.append(len(solution[agent]))

        print('modified solution', solution)
        min_len = min(sol_len)
        print('min_len', min_len, 'sol_len', sol_len)

        for i in range(min_len-1):
            for id, bot in enumerate(self.bots):
                servo = 0
                if i == min_len-2 and id == sol_len.index(min_len):
                    servo = 1

                bot_goal = bot.send_goal(solution['agent'+str(id)][i],
                                         solution['agent'+str(id)][i+1],
                                         servo=servo)
                print('i: {}, id: {}, '.format(i, id), bot_goal)

            for bot in self.bots:
                bot.wait_for_result()


if __name__ == '__main__':
    try:
        rospy.init_node('automata')
        rospack = rospkg.RosPack()
        path = rospack.get_path('grid_phase2_controller') + "/data/"
        induct_station_pose = [(0, 4), (0, 9)]
        automata = Automata(2, induct_station_pose,
                            path+"Sample Data - Sheet1.csv", path+"input.yaml")
        automata.execute()
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
