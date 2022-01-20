#! /usr/bin/env python3

import cv2
import csv
import yaml
import math
import rospy
import rospkg
import argparse
import actionlib

from grid_phase2_controller.msg import botAction, botGoal
from camera_driver.msg import GridPoseArray
from cv_bridge import CvBridge, CvBridgeError
from cbs import solve
# from visualizer import Visualizer


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
        self.__pakage_list = []
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
                    self.__pakage_list.append(row[0])

    def pop_pkg(self) -> int:
        return self.__pakage_list.pop(0)

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

    def __repr__(self) -> str:
        '''
        This function returns the string representation of the object
        params: None
        return: (str): string representation of the object
        '''
        return "InductStation({}, {}): IsOccupied: {}".format(self.id, self.__pose, self.__is_occupied)

    def __getitem__(self, key: str) -> list:
        '''
        This funtion is called when obj[key] is executed
        params: key (str): drop location name
        return: list (list): drop location position
        '''
        pose = [self.__drop_pose[key][0], self.__drop_pose[key][1]]
        dirn = [pose[0], pose[1]+1]
        if self.id % 2 == 1:
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

    def set_occupied(self, occupied: bool) -> None:
        '''
        This function sets the occupied flag of the induct station
        params: occupied (bool): value to set the flag
        return: None
        '''
        self.__is_occupied = occupied

    def is_occupied(self) -> bool:
        '''
        This function returns the occupied flag of the induct station
        params: None
        return: (bool): occupied flag
        '''
        return self.__is_occupied

    def get_pose(self) -> tuple:
        '''
        This function returns pose of the induct station
        params: None
        return: (tuple): pose and dummy pose for direction
        '''
        x, y = self.__pose

        if self.__is_occupied:
            return (x + 4, y), (x + 3, y)

        return (x, y), (x + 1, y)


class Robot:
    def __init__(self, id, goal_station, debug=False):
        self.id = id
        self.client = actionlib.SimpleActionClient(
            'grid_robot_{}'.format(id), botAction)
        self.debug = debug
        self.goal_station = goal_station
        self.current_package = goal_station.pop_pkg()
        self.state = 'picking'
        self.curr_pose, self.goal_pose, self.goal_dirn = None, None, None

        if not self.debug:
            self.client.wait_for_server()

    def send_goal(self, goal_i, goal_j=None, phi=0, servo=0):
        if goal_j:
            phi = math.degrees(math.atan2(goal_i['y'] - goal_j['y'],
                                          goal_j['x'] - goal_i['x']))

        bot_goal = botGoal(x=goal_i['x'], y=goal_i['y'], phi=phi, servo=servo)
        if not self.debug:
            self.client.send_goal(bot_goal)

    def wait_for_result(self):
        if not self.debug:
            self.client.wait_for_result()

    def __repr__(self):
        return "Robot {}: state: {}, curr_pose: {}, goal_pose: {}, goal_dirn: {}".format(
            self.id, self.state, self.curr_pose, self.goal_pose, self.goal_dirn)


class Automata:
    def __init__(self, num_bots, stations, csv_path, yaml_path, debug=False):
        self.stations = [InductStation(i+1, stations[i], csv_path)
                         for i in range(len(stations))]
        self.bots = {i: Robot(i+1+i//2, self.stations[i % len(self.stations)], debug)
                     for i in range(num_bots)}
        # self.viz = Visualizer()
        self.total_pkg = sum([len(i) for i in self.stations])
        self.total_dropped = 0
        self.cv_bridge = CvBridge()
        rospy.Subscriber('grid_robot/poses', GridPoseArray, self.callback)

        with open(yaml_path, 'r') as param_file:
            try:
                self.param = yaml.load(param_file, Loader=yaml.FullLoader)
                self.param['agents'] = []
            except yaml.YAMLError as exc:
                print(exc)

    def callback(self, msg):
        try:
            self.image = self.cv_bridge.imgmsg_to_cv2(msg.image,
                                                      desired_encoding='bgr8')
            for id, x, y, theta in msg.poses:
                pkg_name = self.bots[id-1].current_package
                cv2.putText(self.image, pkg_name, (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cv2.imshow('live_feed', self.image)
                cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

    def automata(self, num_pkg=None):
        len_bots, len_stns = len(self.bots), len(self.stations)

        num_iter = math.ceil(len_bots/len_stns)
        for i in range(num_iter):
            for id in range(i * len_stns, (i + 1) * len_stns, 1):
                if id >= len_bots:
                    continue
                station = self.stations[id % len_stns]
                x, y = station.get_pose()[0]

                self.bots[id].curr_pose = (x, y)
                self.bots[id].goal_station = station
                self.bots[id].goal_pose = [x + 2 * (num_iter - 1 - i), y]
                self.bots[id].goal_dirn = [x + 2 * (num_iter - 1 - i) + 1, y]

                self.param['agents'].append({'start': self.bots[id].curr_pose,
                                             'goal': self.bots[id].goal_pose,
                                             'name': id})
                print('B: {}, S: {}, X: {}, i: {}, id: {}'.format(
                    len_bots, len_stns, num_iter, i, id))

            self.execute()
            # we are executing for specific bots, so reset param for other bots
            self.param['agents'] = []

        # self.viz.flush()

        while not rospy.is_shutdown():
            ## Count no. of pkgs dropped
            self.total_dropped = self.total_pkg - \
                sum([len(i) for i in self.stations]) - len_stns
            if self.total_pkg == self.total_dropped or num_pkg == self.total_dropped:
                rospy.signal_shutdown('killed')

            ## Get New Goals
            for bot in self.bots.values():
                if bot.state == 'dropped':
                    if not bot.goal_station.is_occupied():
                        bot.goal_pose, bot.goal_dirn = bot.goal_station.get_pose()
                        bot.goal_station.set_occupied(True)
                        bot.state = 'picking'
                        print(bot.id, "has dropped", bot.current_package)
                        bot.current_package = bot.goal_station.pop_pkg()
                    else:
                        bot.state = 'standingby'
                        # get induct station pose
                        bot.goal_pose, bot.goal_dirn = bot.goal_station.get_pose()

                elif bot.state == 'picked':
                    # get new goal from the list
                    bot.goal_pose, bot.goal_dirn = bot.goal_station[bot.goal_station.pop(
                    )]
                    bot.state = 'dropping'
                    print(bot.id, "is handling", bot.current_package)

                elif bot.state == 'standby':
                    if not bot.goal_station.is_occupied():
                        # get induct station pose if available
                        bot.goal_pose, bot.goal_dirn = bot.goal_station.get_pose()
                        bot.goal_station.set_occupied(True)
                        bot.state = 'picking'
                    else:
                        continue

                self.param['agents'].append({'start': bot.curr_pose,
                                             'goal': bot.goal_pose,
                                             'name': bot.id//2})

                print(bot)

            print("")
            for stn in self.stations:
                print(stn)
            print("")

            self.execute()
            self.param['agents'].clear()
            # self.viz.flush()

    def execute(self):
        ## CBS Path Planning
        solution, _ = solve(self.param)

        ## Add dummy pose
        for agent in solution:
            # dummy goal position (for phi calc)
            x, y = self.bots[agent].goal_dirn
            # append dummy goal to solution list
            solution[agent].append({'t': len(solution[agent]), 'x': x, 'y': y})

        ## Send Goals
        i, preempted = 0, False
        while not preempted:
            for agent in solution:
                servo = 0

                if i == len(solution[agent])-2:
                    preempted = True
                    if self.bots[agent].state == 'dropping':
                        servo = 1
                        self.bots[agent].state = 'dropped'
                    elif self.bots[agent].state == 'picking':
                        self.bots[agent].state = 'picked'
                        self.bots[agent].goal_station.set_occupied(False)
                    elif self.bots[agent].state == 'standingby':
                        self.bots[agent].state = 'standby'

                self.bots[agent].send_goal(solution[agent][i],
                                           solution[agent][i+1],
                                           servo=servo)

                self.bots[agent].curr_pose = (
                    solution[agent][i]['x'], solution[agent][i]['y'])
                # self.viz.show(solution[agent][i], solution[agent][i+1], agent)
                # self.viz.legend(['IS{}: {}'.format(i.id, len(
                #     i)) for i in self.stations] + ['Drop: {}'.format(self.total_dropped)])

            for agent in solution:
                self.bots[agent].wait_for_result()

            i += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--num_pkg', type=int,
                        help='iterate for num packages, default: None, for all')
    parser.add_argument('-b', '--bots', type=int, default=2,
                        help='number of robots, default: 2')
    parser.add_argument('-c', '--csv', type=str,
                        help='path to csv file, default: None')
    parser.add_argument('-y', '--yaml', type=str,
                        help='path to yaml file, default: None')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='run action client in dubugging mode, do not wait for server')

    args = parser.parse_args()

    try:
        rospy.init_node('automata')
        rospack = rospkg.RosPack()
        path = rospack.get_path('grid_phase2_controller') + "/data/"

        csv_path = args.csv if args.csv else path + "Sample Data - Sheet1.csv"
        yaml_path = args.yaml if args.yaml else path + "input.yaml"

        stations_pose = [(0, 9), (0, 4)]
        automata = Automata(args.bots, stations_pose,
                            csv_path, yaml_path, args.debug)
        automata.automata(args.num_pkg)

        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
