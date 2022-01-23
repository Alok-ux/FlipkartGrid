#! /usr/bin/env python3

import cv2
import rospkg
import numpy as np


class Visualizer:
    def __init__(self):
        self.path = rospkg.RosPack().get_path('grid_phase2_controller') + "/data/arena.jpeg"
        self.image = cv2.imread(self.path)
        self.cell_size = 52.4
        self.x_max, self.y_max = 15, 14
        self.color = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255), (255, 0, 255), (255, 255, 0), (255, 255, 255)]

    def __del__(self):
        cv2.destroyAllWindows()

    def show_plan(self, param, delay=0):
        for agent in param:
            start = self.idx2px(agent['start'][0], agent['start'][1]) 
            goal = self.idx2px(agent['goal'][0], agent['goal'][1])
            cv2.drawMarker(self.image, start, self.color[agent['name']], 1, 25, 2)
            cv2.drawMarker(self.image, goal, self.color[agent['name']], 2, 25, 2)
            cv2.imshow("image", self.image)
            cv2.waitKey(delay)

    def show(self, curr_pos, next_pos, id=0, delay=800):
        if isinstance(curr_pos, dict) and isinstance(next_pos, dict):
            curr_pos = self.idx2px(curr_pos['x'], curr_pos['y'])
            next_pos = self.idx2px(next_pos['x'], next_pos['y'])
        else:
            curr_pos = self.idx2px(curr_pos[0], curr_pos[1])
            next_pos = self.idx2px(next_pos[0], next_pos[1])
        cv2.arrowedLine(self.image, curr_pos, next_pos, self.color[id], 2)
        cv2.imshow("image", self.image) 
        cv2.waitKey(delay)

    def flush(self):
        self.image = cv2.imread(self.path)

    def legend(self, kwargs):
        for i, item in enumerate(kwargs):
            x, y = int(self.image.shape[1] - self.cell_size*2), int(self.cell_size//2*(1 + i))
            cv2.putText(self.image, item, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
        cv2.imshow("image", self.image)
        cv2.waitKey(1)

    def idx2px(self, x, y):
        tx = int(self.cell_size * x + self.cell_size / 2)
        ty = int(self.cell_size * (self.y_max-1-y) + self.cell_size / 2)
        return tx, ty 

    def show_grid(self):
        for i in range(self.x_max):
            for j in range(self.y_max):
                tx, ty = self.idx2px(i, j)
                # cv2.circle(self.image, (tx, ty), 2, (0, 0, 255), -1)
                cv2.putText(self.image, '{},{}'.format(i, j), (tx-15, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        cv2.imshow("image", self.image)
        cv2.waitKey(0)

if __name__ == '__main__':
    vis = Visualizer()
    vis.show_grid()