#! /usr/bin/env python3

import cv2
import csv
import yaml
import time
import math
import rospy
import rospkg
import argparse
import actionlib
from datetime import datetime

from grid_phase2_controller.msg import botAction, botGoal
from camera_driver.msg import GridPoseArray
from cv_bridge import CvBridge, CvBridgeError
from cbs import cbs_record
from visualizer import Visualizer


def read_csv(path: str) -> None:
        '''
        reads csv file containing package list and corresponding drop location
        params: path (str): path of the csv file
        return: None
        '''
        with open(path) as file:
            reader = csv.reader(file)
            data = []
            for i, row in enumerate(reader):
                if i == 0:
                    continue
                data.append(row)
            return data


class DropLocation:
    def __init__(self, name: str, x: int, y: int) -> None:
        self.__name = name
        pose_list = [((x+0, y+0), (x+0, y+1)), 
                    #  ((x+1, y+0), (x+1, y+1)),
                    #  ((x+2, y+1), (x+1, y+1)),
                    #  ((x+2, y+2), (x+1, y+2)),
                     ((x+1, y+3), (x+1, y+2)),
                    #  ((x+0, y+3), (x+0, y+2)),
                    #  ((x-1, y+2), (x+0, y+2)),
                    #  ((x-1, y+1), (x+0, y+1)),
                    ]

        self.__pose = {p: None for p in pose_list}

    def __call__(self, dropped_pose=None):
        if dropped_pose and dropped_pose in self.__pose:
            self.__pose[dropped_pose] = True
        else:
            for p in self.__pose:
                if self.__pose[p]:
                    self.__pose[p] = False
                    return p

    def __str__(self) -> str:
        return "DropLocation: {} {}".format(self.__name, [x for i, x in enumerate(self.__pose) if self.__available[i]])

    def __repr__(self) -> str:
        return self.__str__()

    def get_pose(self, bot) -> tuple:
        for pose in self.__pose:
            if self.__pose[pose] == None:
                self.__pose[pose] = bot
                return pose

    def pop(self, bot):
        for pose in self.__pose:
            if bot == self.__pose[pose]:
                self.__pose[pose] = None
        
class InductStation:
    def __init__(self, id: int, pose: tuple, pkg_path: str) -> None:
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

        data = read_csv(pkg_path)   # read package data from csv file
        self.__package_list = [drop_location for _, idx, drop_location in data if int(idx) == self.id] # packages available at the induct station
        self.__package_names = [pkg for pkg, idx, _ in data if int(idx) == self.id]

    def __len__(self) -> int:
        '''
        This function is called when len(obj) is executed
        params: None
        return: (int): length of the induct list
        '''
        return len(self.__package_list)

    def __iter__(self) -> iter:
        '''
        This function is called when iter(obj) is executed
        params: None
        return: (iter): iterator of induct list
        '''
        return iter(self.__package_list)

    def __repr__(self) -> str:
        '''
        This function returns the string representation of the object
        params: None
        return: (str): string representation of the object
        '''
        return "InductStation({}, {}): IsOccupied: {}".format(self.id, self.__pose, self.__is_occupied)

    def get_package_name(self) -> int:
        '''
        This function pops an item name from the beginning of the package list
        params: None
        return: (int): poped name from the beginning
        '''
        return self.__package_names.pop(0)

    def pop(self) -> int:
        '''
        This function pops an entry from the beginning of the package list
        params: None
        return: (int): poped entry from the beginning
        '''
        return self.__package_list.pop(0)

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
            # return (x + 4, y), (x + 3, y)
            return ((1, 0), (1, 1)) if self.id % 2 == 0 else ((1, 12), (1, 11))

        return (x, y), (x + 1, y)


class Robot:
    def __init__(self, id, goal_station, debug=False):
        self.id = id
        self.client = actionlib.SimpleActionClient(
            'grid_robot_{}'.format(id), botAction)
        self.debug = debug
        self.goal_station = goal_station
        self.current_package = goal_station.get_package_name()
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

    def on_drop(self):
        self.drop_location.pop(self)
        self.drop_location = None        

    def on_pickup(self, drop_location):
        self.drop_location = drop_location
        self.goal_pose, self.goal_dirn = self.drop_location.get_pose(self)

    def wait_for_result(self):
        if not self.debug:
            self.client.wait_for_result()

    def __repr__(self):
        return "Robot {}: state: {}, curr_pose: {}, goal_pose: {}, goal_dirn: {}, pkg: {}".format(
            self.id, self.state, self.curr_pose, self.goal_pose, self.goal_dirn, self.current_package)


class Automata:
    def __init__(self, bots, stations, pkg_path, drop_path, param_path, debug=False):
        self.stations = [InductStation(i+1, stations[i], pkg_path)
                         for i in range(len(stations))]
        self.bots = {bots[i]: Robot(bots[i], self.stations[i % len(self.stations)], debug)
                     for i in range(len(bots))}
        # self.viz = Visualizer()
        self.total_pkg = sum([len(i) for i in self.stations])
        self.total_dropped = 0
        self.cv_bridge = CvBridge()
        self.start_time = datetime.now()

        data = read_csv(drop_path)      # read drop location data from csv file
        self.drop_locations = {drop_location: DropLocation(drop_location, int(x), int(y)) for drop_location, x, y in data}

        rospy.Subscriber('grid_robot/poses', GridPoseArray, self.callback)

        with open(param_path, 'r') as param_file:
            try:
                self.param = yaml.load(param_file, Loader=yaml.FullLoader)
                self.param['agents'] = []
            except yaml.YAMLError as exc:
                print(exc)

    def callback(self, msg):
        try:
            self.image = self.cv_bridge.imgmsg_to_cv2(msg.image,
                                                      desired_encoding='bgr8')
            for pose in msg.poses:
                pkg_name = self.bots[pose.id].current_package
                cv2.putText(self.image, pkg_name, (int(pose.x), int(pose.y)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # cv2.putText(self.image, str(datetime.now() - self.start_time),
            #             (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

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

                bot = list(self.bots.values())
                print(bot)
                bot[id].curr_pose = (x, y)
                bot[id].goal_station = station
                bot[id].goal_pose = [x + 2 * (num_iter - 1 - i), y]
                bot[id].goal_dirn = [x + 2 * (num_iter - 1 - i) + 1, y]

                self.param['agents'].append({'start': bot[id].curr_pose,
                                             'goal': bot[id].goal_pose,
                                             'name': bot[id].id})
                print('B: {}, S: {}, X: {}, i: {}, id: {}'.format(
                    len_bots, len_stns, num_iter, i, id))

            self.execute()
            # we are executing for specific bots, so reset param for other bots
            self.param['agents'] = []

        # self.viz.flush()

        while not rospy.is_shutdown():
            print("\n****************************************************************************\n")
            ## Count no. of pkgs dropped
            self.total_dropped = self.total_pkg - sum([len(i) for i in self.stations]) - len_stns
            if self.total_pkg == self.total_dropped or num_pkg == self.total_dropped:
                rospy.signal_shutdown('killed')

            ## Get New Goals
            for bot in self.bots.values():
                if bot.state == 'dropped':
                    if not bot.goal_station.is_occupied():
                        bot.goal_pose, bot.goal_dirn = bot.goal_station.get_pose()
                        bot.goal_station.set_occupied(True)
                        bot.state = 'picking'
                        bot.current_package = ""
                        
                    else:
                        bot.state = 'standingby'
                        bot.current_package = ""
                        # get induct station pose
                        bot.goal_pose, bot.goal_dirn = bot.goal_station.get_pose()

                elif bot.state == 'standingby' and not bot.goal_station.is_occupied():
                    bot.goal_pose, bot.goal_dirn = bot.goal_station.get_pose()
                    bot.goal_station.set_occupied(True)
                    bot.state = 'picking'
                    bot.current_package = ""

                elif bot.state == 'picked':
                    # get new goal from the list
                    bot.on_pickup(self.drop_locations[bot.goal_station.pop()])
                    bot.state = 'dropping'
                    bot.current_package = bot.goal_station.get_package_name()

                elif bot.state == 'standby':
                    if not bot.goal_station.is_occupied():
                        # get induct station pose if available
                        bot.goal_pose, bot.goal_dirn = bot.goal_station.get_pose()
                        bot.goal_station.set_occupied(True)
                        bot.state = 'picking'
                        if bot.curr_pose in self.param['map']['obstacles']:
                            self.param['map']['obstacles'].pop(self.param['map']['obstacles'].index(bot.curr_pose))
                    else:
                        if bot.curr_pose not in self.param['map']['obstacles']:
                            self.param['map']['obstacles'].append(bot.curr_pose)
                        continue

                self.param['agents'].append({'start': bot.curr_pose,
                                             'goal': bot.goal_pose,
                                             'name': str(bot.id)})

                print(bot)

            print("")
            for stn in self.stations:
                print(stn)
            print("")

            self.execute()
            self.param['agents'].clear()
            # self.viz.flush()

    def execute(self):
        # CBS Path Planning
        print(self.param)
        # self.viz.show_plan(self.param['agents'])
        solution = None
        i = 0
        while not solution and not rospy.is_shutdown():
            solution, _ = cbs_record(self.param)
            time.sleep(0.1)
            i += 1
            if i == 50:
                input()
                break
            # print(solution)
        
        # Add dummy pose
        if not rospy.is_shutdown():
            for agent in solution:
                # dummy goal position (for phi calc)
                x, y = self.bots[int(agent)].goal_dirn
                # append dummy goal to solution list
                solution[agent].append({'t': len(solution[agent]), 'x': x, 'y': y})

        # Send Goals
        i, preempted = 0, False
        while not preempted and not rospy.is_shutdown():
            for agent in solution:
                servo = 0

                if i == len(solution[agent])-2:
                    preempted = True
                    if self.bots[int(agent)].state == 'dropping':
                        servo = 1
                        self.bots[int(agent)].state = 'dropped'
                        self.bots[int(agent)].on_drop()
                    elif self.bots[int(agent)].state == 'picking':
                        self.bots[int(agent)].state = 'picked'
                        self.bots[int(agent)].goal_station.set_occupied(False)
                    elif self.bots[int(agent)].state == 'standingby':
                        self.bots[int(agent)].state = 'standby'

                self.bots[int(agent)].send_goal(solution[agent][i],
                                           solution[agent][i+1],
                                           servo=servo)

                self.bots[int(agent)].curr_pose = (solution[agent][i]['x'], solution[agent][i]['y'])
                # self.viz.show(solution[agent][i], solution[agent][i+1], agent)
                # self.viz.legend(['IS{}: {}'.format(i.id, len(i)) for i in self.stations] + ['Drop: {}'.format(self.total_dropped)])

            for agent in solution:
                self.bots[int(agent)].wait_for_result()

            i += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--num_pkg', type=int,
                        help='iterate for num packages, default: None, for all')
    parser.add_argument('-b', '--bots', nargs='+', type=int, default=2,
                        help='number of robots, default: 2')
    parser.add_argument('-p', '--pkg', type=str,
                        help='path to package file, default: None')
    parser.add_argument('-d', '--drop', type=str,
                        help='path to drop file, default: None')
    parser.add_argument('-y', '--yaml', type=str,
                        help='path to yaml file, default: None')
    parser.add_argument('--debug', action='store_true',
                        help='run action client in dubugging mode, do not wait for server')

    args = parser.parse_args()

    try:
        rospy.init_node('automata')
        rospack = rospkg.RosPack()
        path = rospack.get_path('grid_phase2_controller') + "/data/"

        pkg_path = args.pkg if args.pkg else path + "Sample Data - Sheet1.csv"
        drop_path = args.drop if args.drop else path + "drop_location.csv"
        yaml_path = args.yaml if args.yaml else path + "input.yaml"

        stations_pose = [(0, 9), (0, 4)]
        automata = Automata(args.bots, stations_pose,
                            pkg_path, drop_path, yaml_path, args.debug)
        automata.automata(args.num_pkg)

        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
