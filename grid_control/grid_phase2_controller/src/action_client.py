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
from visualizer import Visualizer

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

        self.__drop_pose = {'Mumbai': [3, 9],
                            'Delhi': [7, 9],
                            'Kolkata': [11, 9],
                            'Chennai': [3, 5],
                            'Bengaluru': [7, 5],
                            'Hyderabad': [11, 5],
                            'Pune': [3, 1],
                            'Ahmedabad': [7, 1],
                            'Jaipur': [11, 1]}   # position of drop

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
        pose = self.__drop_pose[key]
        dirn = [pose[0], pose[1]+1]
        if self.id % 2 == 0:
            pose[1] += 3
            dirn[1] += 1
        return [pose, dirn]

    def pop(self) -> int:
        '''
        This function pops an entry from the beginning of the induct list
        params: None
        return: (int): poped entry from the beginning
        '''
        return self.__induct_list.pop(0)


class Robot:
    def __init__(self, id, debug=False):
        self.client = actionlib.SimpleActionClient(
            'grid_robot_{}'.format(id), botAction)
        if not debug:
            self.client.wait_for_server()
        self.induct_to_goal = True

    def send_goal(self, goal_i, goal_j=None, phi=0, servo=0):
        if goal_j:
            phi = math.degrees(math.atan2(goal_i['y'] - goal_j['y'],
                                          goal_j['x'] - goal_i['x']))

        bot_goal = botGoal(x=goal_i['x'], y=goal_i['y'], phi=phi, servo=servo)
        # return 'x: {}, y: {}, phi: {}, servo: {}'.format(goal_i['x'], goal_i['y'], phi, servo)
        self.client.send_goal(bot_goal)


class Automata:
    def __init__(self, num_bots, induct_station, csv_path, yaml_path, debug=False):
        self.induct_station = [InductStation(i+1, induct_station[i], csv_path)
                               for i in range(len(induct_station))]
        self.bots = [Robot(i+2, debug) for i in range(num_bots)]
        self.viz = Visualizer()
        self.debug = debug

        with open(yaml_path, 'r') as param_file:
            try:
                self.param = yaml.load(param_file, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

    def execute(self, num_pkg=-1):
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
                    self.viz.show(solution['agent'+str(id)][i], solution['agent'+str(id)][i+1], id)
                    self.viz.legend(['IS1: {}'.format(len(self.induct_station[0])), 'IS2: {}'.format(len(self.induct_station[1]))])
                print("")

                if not self.debug:
                    for bot in self.bots:
                        bot.client.wait_for_result()
                
            ## Get New Goals
            current_index = i
            for id, bot in enumerate(self.bots):
                self.param['agents'][id]['start'] = tuple(solution['agent'+str(id)][current_index].values())[1:]
                if id == sol_len.index(min_len):
                    if bot.induct_to_goal:
                        goal_name = self.induct_station[id].pop()                               
                        goal_pose[id] = self.induct_station[id][goal_name]
                    else:
                        # goal_pose[id] = [list(self.induct_station[id].pose)] * 2
                        # goal_pose[id][1][1] += 1              
                        goal_pose[id][0] = self.induct_station[id].pose
                        goal_pose[id][1] = [self.induct_station[id].pose[0], self.induct_station[id].pose[1]]
                    self.param['agents'][id]['goal'] = goal_pose[id][0]
                else:
                    self.param['agents'][id]['goal'] = tuple(solution['agent'+str(id)][-2].values())[1:]

            print('params: ', self.param['agents'])
            self.viz.flush()
            if iterations == 50:
                rospy.signal_shutdown('killed')
                

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--num_pkg', type=int, default=-1, help='iterate for num packages, default: -1, for all')
    parser.add_argument('-b', '--bots', type=int, default=2, help='number of robots, default: 2')
    parser.add_argument('-i', '--induct_stations', type=int, default=2, help='number of induct stations, default: 2')
    parser.add_argument('-c', '--csv', type=str, help='path to csv file, default: None')
    parser.add_argument('-y', '--yaml', type=str, help='path to yaml file, default: None')
    parser.add_argument('-d', '--debug', action='store_true', help='run action client in dubugging mode, do not wait for server')

    args = parser.parse_args()

    try:
        rospy.init_node('automata')
        rospack = rospkg.RosPack()
        path = rospack.get_path('grid_phase2_controller') + "/data/"

        csv_path = args.csv if args.csv else path + "Sample Data - Sheet1.csv" 
        yaml_path = args.yaml if args.yaml else path + "input.yaml"
        
        induct_station_pose = [(0, 4), (0, 9)]
        automata = Automata(2, induct_station_pose, csv_path, yaml_path, args.debug)
        automata.execute()
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
