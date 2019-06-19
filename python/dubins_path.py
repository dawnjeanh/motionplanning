#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import math
from numpy import arange
from copy import deepcopy

def draw_point(point, arrow_length=0.5):
    plt.plot(point[0], point[1], 'o')
    plt.arrow(point[0], point[1], arrow_length * math.cos(point[2]), arrow_length * math.sin(point[2]), head_width=0.05)

class DubinsPath(object):
    def __init__(self, start, end, r=1.0):
        self._s = start
        self._e = end
        self._paths = []

    def calc_paths(self):
        le = self.calc_end()
        types = [self.calc_lsl_from_origin,
                 self.calc_rsr_from_origin,
                 self.calc_lsr_from_origin,
                 self.calc_rsl_from_origin,
                 self.calc_rlr_from_origin,
                 self.calc_lrl_from_origin]
        for t in types:
            path = t(le)
            if path:
                self._paths.append(path)
        return self._paths

    def get_shortest_path(self):
        shortest_cost = float("inf")
        shortest_path = []
        for path in self._paths:
            cost = path[0][1] + path[1][1] + path[2][1]
            if cost < shortest_cost:
                shortest_path = path
                shortest_cost = cost
        return deepcopy(shortest_path)

    def calc_end(self):
        ex = self._e[0] - self._s[0]
        ey = self._e[1] - self._s[1]

        lex = math.cos(self._s[2]) * ex + math.sin(self._s[2]) * ey
        ley = - math.sin(self._s[2]) * ex + math.cos(self._s[2]) * ey
        leyaw = self._e[2] - self._s[2]
        return [lex, ley, leyaw]

    def mod2pi(self, theta):
        return theta - 2.0 * math.pi * math.floor(theta / 2.0 / math.pi)

    def calc_lsl_from_origin(self, e):
        x_ = e[0] - math.sin(e[2])
        y_ = e[1] - 1 + math.cos(e[2])

        u = math.sqrt((x_) ** 2 + (y_) ** 2)
        t = self.mod2pi(math.atan2(y_ , x_))
        v = self.mod2pi(e[2] - t)
        return [['l', t], ['s', u], ['l', v]]

    def calc_rsr_from_origin(self, e):
        e_ = deepcopy(e)
        e_[1] = -e_[1]
        e_[2] = self.mod2pi(-e_[2])

        path = self.calc_lsl_from_origin(e_)
        path[0][0] = 'r'
        path[2][0] = 'r'
        return path

    def calc_lsr_from_origin(self, e):
        x_ = e[0] + math.sin(e[2])
        y_ = e[1] - 1 - math.cos(e[2])
        u1_square = x_ ** 2 + y_ ** 2
        if u1_square < 4:
            return []
        t1 = self.mod2pi(math.atan2(y_, x_))
        u = math.sqrt(u1_square - 4)
        theta = self.mod2pi(math.atan(2 / u))
        t = self.mod2pi(t1 + theta)
        v = self.mod2pi(t - e[2])
        return [['l', t], ['s', u], ['r', v]]

    def calc_rsl_from_origin(self, e):
        e_ = deepcopy(e)
        e_[1] = -e_[1]
        e_[2] = self.mod2pi(-e_[2])

        path = self.calc_lsr_from_origin(e_)
        if path:
            path[0][0] = 'r'
            path[2][0] = 'l'
            return path
        else:
            return []

    def calc_lrl_from_origin(self, e):
        x_ = e[0] - math.sin(e[2])
        y_ = e[1] - 1 + math.cos(e[2])
        u1 = math.sqrt(x_ ** 2 + y_ ** 2)
        if u1 > 4:
            return []
        t1 = math.atan2(y_, x_)
        theta = math.acos(u1 / 4)
        t = self.mod2pi(math.pi / 2 + t1 + theta)
        u = self.mod2pi(math.pi + 2 * theta)
        v = self.mod2pi(math.pi / 2 - t1 + theta + e[2])
        return [['l', t], ['r', u], ['l', v]]

    def calc_rlr_from_origin(self, e):
        e_ = deepcopy(e)
        e_[1] = -e_[1]
        e_[2] = self.mod2pi(-e_[2])

        path = self.calc_lrl_from_origin(e_)
        if path:
            path[0][0] = 'r'
            path[1][0] = 'l'
            path[2][0] = 'r'
            return path
        else:
            return []

    @classmethod
    def gen_path(cls, s, path, r=1.0):
        def calc_TurnCenter(point, dir='l', r=1.0):
            if dir == 'l':
                ang = point[2] + math.pi / 2
            elif dir == 'r':
                ang = point[2] - math.pi / 2
            else:
                return None
            x = point[0] + math.cos(ang)
            y = point[1] + math.sin(ang)
            return (x, y)
        r_x = []
        r_y = []
        ps_x = []
        ps_y = []
        start = s
        yaw = s[2]
        for p in path:
            if p[0] == 's':
                for l in arange(0, p[1], 0.5):
                    ps_x.append(start[0] + math.cos(yaw) * l)
                    ps_y.append(start[1] + math.sin(yaw) * l)
                ps_x.append(start[0] + math.cos(yaw) * p[1])
                ps_y.append(start[1] + math.sin(yaw) * p[1])
                r_x.append(ps_x)
                r_y.append(ps_y)
            else:
                center = calc_TurnCenter(start, p[0])
                ang_start = math.atan2(start[1] - center[1], start[0] - center[0])
                ang_end = ang_start + p[1] if p[0] == 'l' else ang_start - p[1]
                for ang in arange(ang_start, ang_end, 0.05 if p[0] == 'l' else -0.05):
                    ps_x.append(center[0] + math.cos(ang) * r)
                    ps_y.append(center[1] + math.sin(ang) * r)
                ps_x.append(center[0] + math.cos(ang_end) * r)
                ps_y.append(center[1] + math.sin(ang_end) * r)
                r_x.append(ps_x)
                r_y.append(ps_y)
                yaw = start[2] + p[1] if p[0] == 'l' else start[2] - p[1]
            start = (ps_x[-1], ps_y[-1], yaw)
            ps_x = []
            ps_y = []
        return r_x, r_y

def test1():
    for i in range(0, 360, 30):
        plt.figure()
        start = [1, 1, 90 / 180.0 * math.pi]
        end = [3, 0, i / 180.0 * math.pi]
        dubins = DubinsPath(start, end)
        paths = dubins.calc_paths()
        print paths
        for i, path in enumerate(paths):
            plt.subplot(2, 3, i+1)
            plt.title('{}{}{}'.format(path[0][0], path[1][0], path[2][0]))
            draw_point(start)
            draw_point(end)
            xs, ys = DubinsPath.gen_path(start, path)
            for i in range(3):
                plt.plot(xs[i], ys[i])
            plt.axis("equal")
        plt.show()

def test2():
    plt.figure()
    for i in range(12):
        start = [1, 1, 300 / 180.0 * math.pi]
        end = [-2, 0, i * 30 / 180.0 * math.pi]
        dubins = DubinsPath(start, end)
        dubins.calc_paths()
        path = dubins.get_shortest_path()
        print path
        plt.subplot(3, 4, i+1)
        plt.title('{}{}{}'.format(path[0][0], path[1][0], path[2][0]))
        draw_point(start)
        draw_point(end)
        xs, ys = DubinsPath.gen_path(start, path)
        for i in range(3):
            plt.plot(xs[i], ys[i])
        plt.axis("equal")
    plt.show()

if __name__ == "__main__":
    test1()
    # test2()