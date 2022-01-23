#!/usr/bin/env python3

from cbs import solve
import json

path = '/home/lucifer/flipkart_ws/src/FlipkartGrid/grid_control/grid_phase2_controller/data/solutions.json'

param = {'agents': [{'start': (0, 9), 'goal': (3, 5), 'name': '1'}, {'start': (10, 5), 'goal': (0, 4), 'name': '2'}], 'map': {'dimensions': [15, 13], 'obstacles': [(0, 0), (0, 1), (0, 2), (0, 3), (0, 5), (0, 6), (0, 7), (0, 8), (0, 10), (0, 11), (0, 12), (0, 13), (3, 2), (3, 3), (4, 2), (4, 3), (3, 6), (3, 7), (4, 6), (4, 7), (3, 10), (3, 11), (4, 10), (4, 11), (7, 2), (7, 3), (8, 2), (8, 3), (7, 6), (7, 7), (8, 6), (8, 7), (7, 10), (7, 11), (8, 10), (8, 11), (11, 2), (11, 3), (12, 2), (12, 3), (11, 6), (11, 7), (12, 6), (12, 7), (11, 10), (11, 11), (12, 10), (12, 11)]}}
solution, env = solve(param)
if solution:
    with open(path, 'r') as file:
        data = json.load(file)
    data.append(solution)
    with open(path, 'w') as file:
        json.dump(data, file)
    print(solution, env)
