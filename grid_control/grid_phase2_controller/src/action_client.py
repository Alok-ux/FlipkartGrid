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
        assert id != 0              # asserts id not equal to 0 as id starts from 1
        self.id = id                # station id e.q. IS1, IS2
        self.__pose = pose          # position of induct station e.q. (1, 2)
        self.__is_occupied = False  # flag for induct station occupancy

        self.__induct_list = []     # packages available at the induct station
        self.__read_csv(path)       # read package data from csv file

        self.__drop_pose = {'Mumbai': (3, 9),
                            'Delhi': (7, 9),
                            'Kolkata': (11, 9),
                            'Chennai': (3, 5),
                            'Bengaluru': (7, 5),
                            'Hyderabad': (11, 5),
                            'Pune': (3, 1),
                            'Ahmedabad': (7, 1),
                            'Jaipur': (11, 1)}   # position of drop

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
        pose = [self.__drop_pose[key][0], self.__drop_pose[key][1]]
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
        self.__is_occupied = False
        return self.__induct_list.pop(0)

    def get_pose(self, set_occupied=True):
        '''
        This function returns pose of the induct station
        params: None
        return: (tuple): pose and dummy pose for direction
        '''
        x, y = self.__pose
        standby = False

        if self.__is_occupied:
            standby = True
            return (x + 4, y), (x + 3, y), standby

        self.__is_occupied = set_occupied
        return (x, y), (x + 1, y), standby


class Robot:
    def __init__(self, id, debug=False):
        self.client = actionlib.SimpleActionClient(
            'grid_robot_{}'.format(id), botAction)
        if not debug:
            self.client.wait_for_server()
        self.induct_to_goal = True
        self.standby = True
        self.goal_station = None
        self.curr_pose = None

    def send_goal(self, goal_i, goal_j=None, phi=0, servo=0):
        if goal_j:
            phi = math.degrees(math.atan2(goal_i['y'] - goal_j['y'],
                                          goal_j['x'] - goal_i['x']))

        bot_goal = botGoal(x=goal_i['x'], y=goal_i['y'], phi=phi, servo=servo)
        self.client.send_goal(bot_goal)


class Automata:
    def __init__(self, num_bots, induct_station, csv_path, yaml_path, debug=False):
        self.induct_station = [InductStation(i+1, induct_station[i], csv_path)
                               for i in range(len(induct_station))]
        self.bots = [Robot(i, debug) for i in range(num_bots)]
        self.viz = Visualizer()

        self.debug = debug
        self.total_pkg = sum([len(i) for i in self.induct_station])
        self.min_path_idx = None

        with open(yaml_path, 'r') as param_file:
            try:
                self.param = yaml.load(param_file, Loader=yaml.FullLoader)
                self.param['agents'] = []
            except yaml.YAMLError as exc:
                print(exc)

    def get_induct_station(self, id):
        cost = []
        for stn in self.induct_station:
            bx, by = self.bots[id].curr_pose
            ix, iy = stn.get_pose()[0]
            cost.append(abs(bx - ix) + abs(by - iy))
        return self.induct_station[cost.index(min(cost))]

    def automata(self, num_pkg=None):
        len_bots, len_stns = len(self.bots), len(self.induct_station)
        num_iter = math.ceil(len_bots/len_stns)
        for i in range(num_iter):
            
            for id in range(i * len_stns, (i + 1) * len_stns, 1):
                if id >= len_bots:
                    continue
                station = self.induct_station[id % len_stns]
                self.bots[id].goal_station = station
                x, y = self.bots[id].curr_pose = station.get_pose(set_occupied=False)[0]

                self.param['agents'].append({'start': self.bots[id].curr_pose,
                                             'goal': [x + 2 * (num_iter - 1 - i), y],
                                             'dirn': [x + 2 * (num_iter - 1 - i) + 1, y],
                                             'name': 'agent' + str(id)})
                print('B: {}, S: {}, X: {}, i: {}, id: {}'.format(len_bots, len_stns, num_iter, i, id))

            print(self.param['agents'])
            self.execute()
            self.param['agents'] = []                                       # we are executing for specific bots, so reset param for other bots

        for i, bot in enumerate(self.bots):
            self.param['agents'].append({'name': 'agent' + str(i)})
            bot.standby = False
            bot.goal_station = self.induct_station[i % len_stns]            # comment this to use manhattan dist based station selection
            
        self.min_path_idx = range(len_bots)
        self.viz.flush()
        
        while not rospy.is_shutdown():
            ## Count no. of pkgs dropped
            total_dropped = self.total_pkg - sum([len(i) for i in self.induct_station]) - len(self.induct_station)
            if self.total_pkg == total_dropped or num_pkg == total_dropped:
                rospy.signal_shutdown('killed')

            ## Get New Goals
            for id, bot in enumerate(self.bots):
                self.param['agents'][id]['start'] = bot.curr_pose
                if id in self.min_path_idx:
                    if bot.induct_to_goal:
                        goal, dirn = bot.goal_station[bot.goal_station.pop()]
                    else:
                        # bot.goal_station = self.get_induct_station(id)    # uncomment this to use manhattan dist based station selection
                        goal, dirn, standby = bot.goal_station.get_pose()
                        bot.standby = standby
                    self.param['agents'][id]['goal'] = goal
                    self.param['agents'][id]['dirn'] = dirn

            self.execute()

            self.viz.legend(['IS{}: {}'.format(i.id, len(i)) for i in self.induct_station] + ['Drop: {}'.format(total_dropped)])
            self.viz.flush()

    def execute(self):
        ## CBS Path Planning 
        print('\nparam: ', self.param['agents'])
        solution, _ = solve(self.param)

        ## Add dummy pose
        path_len = []
        for i, agent in enumerate(solution):
            x, y = self.param['agents'][i]['dirn']                                  # dummy goal position (for phi calc)
            solution[agent].append({'t': len(solution[agent]), 'x': x, 'y': y})     # append dummy goal to solution list
            path_len.append(len(solution[agent]))                                   # append len of solution to path_len list

        # print('\nsolution: ', solution)

        min_path = min(path_len)                                                    # find solution list with min path
        self.min_path_idx = [i for i, l in enumerate(path_len) if l == min_path]    # path len list only with items = min path

        # print('\nmin_path: ', min_path, '\tpath_len: ', path_len, '\tmin_idx: ', min_path_idx)
        # print('\nPath follow: \n')

        ## Send Goals
        for i in range(min_path-1):
            for id, bot in enumerate(self.bots):
                agent_id = 'agent'+str(id)
                if agent_id not in solution:
                    continue
                servo = 0
                if i == min_path-2 and id in self.min_path_idx and not bot.standby: # if it is last iteration (i == min_path-2)
                    servo = 1 if bot.induct_to_goal else 0                          # & id == bot_id (which completed the goal)
                    bot.induct_to_goal = not bot.induct_to_goal                     # toggle induct_to_goal

                if not self.debug:
                    bot.send_goal(solution[agent_id][i],
                                  solution[agent_id][i+1],
                                  servo=servo)
                bot.curr_pose = (solution[agent_id][i]['x'], solution[agent_id][i]['y'])
                # print('i: {}, id: {}, pos: {}'.format(i, id, tuple(solution['agent'+str(id)][i].values())[1:]))
                self.viz.show(solution[agent_id][i], solution[agent_id][i+1], id)
            
            # print("")

            if not self.debug:
                for bot in self.bots:
                    bot.client.wait_for_result()
                

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--num_pkg', type=int, help='iterate for num packages, default: None, for all')
    parser.add_argument('-b', '--bots', type=int, default=2, help='number of robots, default: 2')
    # parser.add_argument('-i', '--induct_stations', type=int, default=2, help='number of induct stations, default: 2')
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
        automata = Automata(args.bots, induct_station_pose, csv_path, yaml_path, args.debug)
        automata.automata(args.num_pkg)
        
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
