#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import math
from reeds_shepp_path import ReedsSheppPath, draw_point

class Car(object):
    def __init__(self, l, w):
        self._l = l
        self._w = w
        self._pos = None
        self._outline_x = []
        self._outline_y = []

    def set_position(self, point_dir):
        self._pos = point_dir

    def get_outline(self):
        x = self._pos[0]
        y = self._pos[1]
        yaw = self._pos[2]
        tail_x = x - math.cos(yaw) * self._l / 4
        tail_y = y - math.sin(yaw) * self._l / 4
        tail_l_x = tail_x + math.cos(yaw + math.pi / 2) * self._w / 2
        tail_l_y = tail_y + math.sin(yaw + math.pi / 2) * self._w / 2
        tail_r_x = tail_x + math.cos(yaw - math.pi / 2) * self._w / 2
        tail_r_y = tail_y + math.sin(yaw - math.pi / 2) * self._w / 2
        head_x = x + math.cos(yaw) * self._l * 3 / 4
        head_y = y + math.sin(yaw) * self._l * 3 / 4
        head_l_x = head_x + math.cos(yaw + math.pi / 2) * self._w / 2
        head_l_y = head_y + math.sin(yaw + math.pi / 2) * self._w / 2
        head_r_x = head_x + math.cos(yaw - math.pi / 2) * self._w / 2
        head_r_y = head_y + math.sin(yaw - math.pi / 2) * self._w / 2
        self._outline_x = [tail_l_x, tail_r_x, head_r_x, head_l_x, tail_l_x]
        self._outline_y = [tail_l_y, tail_r_y, head_r_y, head_l_y, tail_l_y]
        return self._outline_x, self._outline_y

    def show(self):
        self.get_outline()
        draw_point(self._pos, self._l / 2.0)
        plt.plot(self._outline_x, self._outline_y, color='black')

if __name__ == "__main__":
    car = Car(2.0, 1.0)
    plt.figure()
    start = [1, 1, math.pi / 3]
    end = [5, 10, -math.pi / 2]
    r = 5.0
    rspath = ReedsSheppPath(start, end, r)
    rspath.calc_paths()
    path, _ = rspath.get_shortest_path()
    print path
    xs, ys, yaws = ReedsSheppPath.gen_path(start, path, r, section=False)
    for i in range(len(xs)):
        plt.clf()
        draw_point(start)
        draw_point(end)
        plt.axis("equal")
        plt.plot(xs, ys)
        car.set_position([xs[i], ys[i], yaws[i]])
        car.show()
        plt.pause(0.1)
    plt.show()