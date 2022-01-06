#!/usr/bin/env python3

import sys
from grid_phase2_controller.msg import Goal, botGoal
from cbs import solve

# sys.path.append('/home/lucifer/flipkart_ws/src/multi_agent_path_planning/centralized')


param = {'agents': [{'start': [0, 4], 'goal': [3, 5], 'name': 'agent0'}, {'start': [0, 9], 'goal': [5, 3], 'name': 'agent1'}], 'map': {'dimensions': [15, 14], 'obstacles': [(0, 0), (0, 1), (0, 2), (0, 3), (0, 5), (0, 6), (0, 7), (0, 8), (0, 10), (0, 11), (0, 12), (0, 13), (3, 2), (3, 3), (4, 2), (4, 3), (3, 6), (3, 7), (4, 6), (4, 7), (3, 10), (3, 11), (4, 10), (4, 11), (7, 2), (7, 3), (8, 2), (8, 3), (7, 6), (7, 7), (8, 6), (8, 7), (7, 10), (7, 11), (8, 10), (8, 11), (11, 2), (11, 3), (12, 2), (12, 3), (11, 6), (11, 7), (12, 6), (12, 7), (11, 10), (11, 11), (12, 10), (12, 11)]}}
solution, _ = solve(param)
# print("param: {} \nsolution: {}".format(param, solution))

for agent in solution:
    goal = botGoal(order = [Goal(t=d['t'], x=d['x'], y=d['y']) for d in solution[agent]])
    print(goal)