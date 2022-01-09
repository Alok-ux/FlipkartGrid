#! /usr/bin/env python3

import csv
import yaml
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

        self.__drop_pose = {'Mumbai': (3, 9),
                            'Delhi': (7, 9),
                            'Kolkata': (11, 9),
                            'Chennai': (3, 5),
                            'Bengaluru': (7, 5),
                            'Hyderabad': (11, 5),
                            'Pune': (3, 1),
                            'Ahmedabad': (7, 1),
                            'Jaipur': (11, 1)}      # position of drop location

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
        # self.client.wait_for_server()
        self.goal = list()

    def send_goal(self):
        x, y = self.goal.pop(0)
        x_next, y_next = self.goal[0]

        bot_goal = botGoal(x=x, y=y, phi=0)
        self.client.send_goal(bot_goal)


class Automata:
    def __init__(self, num_bots, induct_station, data_path, map_path):
        self.induct_station = [InductStation(i+1, induct_station[i], data_path)
                               for i in range(len(induct_station))]
        self.bots = [Robot(i) for i in range(num_bots)]

        with open(map_path, 'r') as param_file:
            try:
                self.param = yaml.load(param_file, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

    def execute(self, num_pkg=-1):
        self.induct_station[0].pop()
        self.induct_station[1].pop()
        self.param['agents'] = list()
        for i in range(len(self.bots)):
            self.param['agents'].append({'start': self.induct_station[i].pose,
                                         'goal': self.induct_station[i][self.induct_station[i].pop()],
                                         'name': 'agent'+str(i)
                                         })
        print(self.param)
        solution, _ = solve(self.param)
        print(solution)

        # TODO: Send goal to server
        for i, bot in enumerate(self.bots):
            bot.goal = [(d['x'], d['y']) for d in solution['agent'+str(i)]]

        while not rospy.is_shutdown():
            try:
                for bot in self.bots:
                    bot.send_goal()
                for bot in self.bots:
                    bot.wait_for_result()
                pass
            except IndexError:
                break


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
