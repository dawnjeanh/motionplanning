#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from mapinfo import MapInfo
import math
from copy import deepcopy
from reeds_shepp_path import ReedsSheppPath, draw_point
from car import Car
from a_star import AStar
import matplotlib.pyplot as plt
import time

class HybridAStar(object):
    def __init__(self, start, end, map_info, car, r):
        self._s = start
        self._e = end
        self._map_info = map_info
        self._car = car
        self._r = r
        self._openset = dict()
        self._closeset = dict()

    def distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def neighbors(self, p):
        step = 5.0
        paths = [['l', step / self._r], ['s', step], ['r', step / self._r],
                 ['l', -step / self._r], ['s', -step], ['r', -step / self._r]]
        for path in paths:
            xs, ys, yaws = ReedsSheppPath.gen_path(p, [path], r=self._r, section=False)
            if not self.is_collision_rs_car(xs, ys, yaws):
                yield (round(xs[-1], 2), round(ys[-1], 2), round(yaws[-1], 2)), path, [xs, ys, yaws]

    def is_collision_rs_car(self, xs, ys, yaws):
        for pt in zip(xs, ys, yaws):
            self._car.set_position(pt)
            if self._map_info.is_collision(car_outline=self._car.get_outline()):
                return True
        return False

    def h_cost(self, s):
        plan = AStar((s[0], s[1]), (self._e[0], self._e[1]), self._map_info)
        if plan.run(display=False):
            path = plan.reconstruct_path()
        d = 0
        for i in range(len(path) - 1):
            d += self.distance(path[i], path[i + 1])
        return d

    def reconstruct_path(self):
        waypoint = []
        xs = []
        ys = []
        yaws = []
        pt = self._e
        while pt != self._s:
            waypoint.append(pt)
            pt = self._closeset[pt]['camefrom']
        for pt in waypoint[::-1]:
            x, y, yaw = self._closeset[pt]['path'][1]
            xs += x
            ys += y
            yaws += yaw
        return xs, ys, yaws

    def run(self, display=False):
        d = self.h_cost(self._s)
        self._openset[self._s] = {'g': 0, 'h': d, 'f': d, 'camefrom':None, 'path': []}
        while self._openset:
            x = min(self._openset, key=lambda key: self._openset[key]['f'])
            self._closeset[x] = deepcopy(self._openset[x])
            del self._openset[x]
            if display:
                self._map_info.close = (x[0], x[1])
            rspath = ReedsSheppPath(x, self._e, self._r)
            rspath.calc_paths()
            path, _ = rspath.get_shortest_path()
            xs, ys, yaws = ReedsSheppPath.gen_path(x, path, self._r, section=False)
            if not self.is_collision_rs_car(xs, ys, yaws):
                self._closeset[self._e] = {'camefrom': x, 'path': [path, [xs, ys, yaws]]}
                return True
            for y, path, line in self.neighbors(x):
                if y in self._closeset:
                    continue
                if display:
                    self._map_info.open = (y[0], y[1])
                tentative_g_score = self._closeset[x]['g'] + (abs(path[1]) if path[0] == 's' else abs(path[1]) * self._r)
                if y not in self._openset:
                    tentative_is_better = True
                elif tentative_g_score < self._openset[y]['g']:
                    tentative_is_better = True
                else:
                    tentative_is_better = False
                if tentative_is_better:
                    d = self.h_cost(y)
                    self._openset[y] = {'g': tentative_g_score, 'h': d, 'f': tentative_g_score+d, 'camefrom': x, 'path': [path, line]}
        return False

def main1():
    m = MapInfo(60, 40)
    car = Car(10.0, 5.0)
    m.show()
    m.start = (10, 10, math.pi / 2)
    m.end = (50, 30, math.pi / 2)
    car.set_position(m.start)
    car.show()
    m.obstacle = [(20, i) for i in range(30)] + [(40, 40 - i) for i in range(30)]
    m.update()
    raw_input('enter to start ...')
    plan = HybridAStar(m.start, m.end, m, car, r=5.0)
    if plan.run(True):
        xs, ys, yaws = plan.reconstruct_path()
        m.path = zip(xs, ys)
        for i in range(len(xs)):
            if i != len(xs) - 1 and i % 10 != 0:
                continue
            plt.cla()
            m.show()
            m.start = (10, 10, math.pi / 2)
            m.end = (50, 30, math.pi / 2)
            m.obstacle = [(20, i) for i in range(30)] + [(40, 40 - i) for i in range(30)]
            m.path = zip(xs, ys)
            car.set_position([xs[i], ys[i], yaws[i]])
            car.show()
            plt.pause(0.1)
    m.wait_close()

def main2():
    m = MapInfo(60, 40)
    car = Car(10.0, 5.0)
    start = (10, 25, 0)
    end = (45, 5, math.pi / 2)
    m.show()
    m.start = start
    m.end = end
    ob = [(40, i) for i in range(15)] + [(50, i) for i in range(15)] + [(i, 15) for i in range(40)]
    m.obstacle = ob
    car.set_position(m.start)
    car.show()
    m.update()
    raw_input('enter to start ...')
    plan = HybridAStar(m.start, m.end, m, car, r=5.0)
    if plan.run(True):
        xs, ys, yaws = plan.reconstruct_path()
        m.path = zip(xs, ys)
        m.update()
        for i in range(len(xs)):
            if i != len(xs) - 1 and i % 10 != 0:
                continue
            plt.cla()
            m.show()
            m.start = start
            m.end = end
            m.obstacle = ob
            m.path = zip(xs, ys)
            car.set_position([xs[i], ys[i], yaws[i]])
            car.show()
            plt.pause(0.1)
    m.wait_close()

def main3():
    m = MapInfo(60, 25)
    car = Car(10.0, 5.0)
    start = (10, 15, 0)
    end = (35, 5, 0)
    m.show()
    m.start = start
    m.end = end
    ob = [(30, i) for i in range(10)] + [(45, i) for i in range(10)] + [(i, 10) for i in range(31)] + [(i+45, 10) for i in range(15)]
    m.obstacle = ob
    car.set_position(m.start)
    car.show()
    m.update()
    raw_input('enter to start ...')
    plan = HybridAStar(m.start, m.end, m, car, r=5.0)
    if plan.run(True):
        xs, ys, yaws = plan.reconstruct_path()
        m.path = zip(xs, ys)
        m.update()
        for i in range(len(xs)):
            if i != len(xs) - 1 and i % 10 != 0:
                continue
            plt.cla()
            m.show()
            m.start = start
            m.end = end
            m.obstacle = ob
            m.path = zip(xs, ys)
            car.set_position([xs[i], ys[i], yaws[i]])
            car.show()
            plt.pause(0.1)
    m.wait_close()

if __name__ == "__main__":
    main3()