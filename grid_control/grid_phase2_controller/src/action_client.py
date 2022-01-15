#! /usr/bin/env python3

import csv
import yaml
import math
import rospy
import rospkg
import argparse
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
        # self.client.wait_for_server()
        self.induct_to_goal = True

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
            goal_name = self.induct_station[i].pop()                                    # pop city name from csv file
            goal_pose.append(self.induct_station[i][goal_name])                         # append city position to goal_pose

            self.param['agents'].append({'start': self.induct_station[i].pose,          # start position
                                         'goal': goal_pose[i][0],                       # valid goal position
                                         'name': 'agent'+str(i)                         # agent id = bot id
                                         })

        iterations = 0
        while not rospy.is_shutdown():
            iterations += 1
            ## CBS Path Planning 
            print('\nparam: ', self.param['agents'])
            solution, _ = solve(self.param)


            ## Add dummy pose
            sol_len = []
            for i, agent in enumerate(solution):
                x, y = goal_pose[i][1][0], goal_pose[i][1][1]                           # dummy goal position (for phi calc)
                solution[agent].append({'t': len(solution[agent]), 'x': x, 'y': y})     # append dummy goal to solution list
                sol_len.append(len(solution[agent]))                                    # append len of soln list to sol_len list

            print('\nsolution: ', solution)
            min_len = min(sol_len)                                                      # find solution list with min length
            print('\nmin_len: ', min_len, '\tsol_len: ', sol_len)

            print('\nPath follow: \n')

            ## Send Goals
            for i in range(min_len-1):
                for id, bot in enumerate(self.bots):
                    servo = 0
                    if i == min_len-2 and id == sol_len.index(min_len):                 # if it is last iteration (i == min_len-2)
                        servo = 1 if bot.induct_to_goal else 0                          # & id == bot_id (which completed the goal)
                        bot.induct_to_goal = not bot.induct_to_goal                     # toggle induct_to_goal

                    bot.send_goal(solution['agent'+str(id)][i],
                                  solution['agent'+str(id)][i+1],
                                  servo=servo)
                    print('i: {}, id: {}, pos: {}'.format(i, id, tuple(solution['agent'+str(id)][i].values())[1:]))

                print("")
                
                # for bot in self.bots:
                #     bot.wait_for_result()
                
            ## Get New Goals
            current_index = i
            for id, bot in enumerate(self.bots):
                self.param['agents'][id]['start'] = tuple(solution['agent'+str(id)][current_index].values())[1:]
                if id == sol_len.index(min_len):
                    if bot.induct_to_goal:
                        goal_name = self.induct_station[id].pop()                               
                        goal_pose[id] = self.induct_station[id][goal_name]
                        self.param['agents'][id]['goal'] = goal_pose[id][0]     
                    else:
                        self.param['agents'][id]['goal'] = self.induct_station[id].pose                     
                else:
                    self.param['agents'][id]['goal'] = tuple(solution['agent'+str(id)][-2].values())[1:]

            print('params: ', self.param['agents'])
            
            if iterations == 3:
                break

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--num_pkg', type=int, default=-1, help='iterate for num packages, default: -1, for all')
    parser.add_argument('-b', '--bots', type=int, default=2, help='number of robots, default: 2')
    parser.add_argument('-i', '--induct_stations', type=int, default=2, help='number of induct stations, default: 2')
    parser.add_argument('-m', '--map', type=str, default='/data/input.yaml', help='path to yaml file, default: relative path to map file')
    parser.add_argument('-d', '--data', type=str, default='/data/Sample Data - Sheet1.csv', help='path to data file, default: relative path to data file')
    args = parser.parse_args()

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
