#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from mapinfo import MapInfo
from random import randint, uniform
import math
from rrt_dubins import RRTDubins as RRTDubinsBase
from rrt_dubins import reconstruct_path
from dubins_path import DubinsPath

class RRTDubins(RRTDubinsBase):
    def __init__(self, q_init, r):
        super(RRTDubins, self).__init__(q_init, r)

    def cost(self, q):
        q_now = q
        c = 0
        while(q_now != self._root):
            q_now, path, _ = self.get_info(q_now)
            for p in path:
                c += p[1] if p[0] == 's' else p[1] * self._r
        return c

    def rewire(self, q_new, r, map_info):
        q_near = [q for q in self._rrt.keys() if distance(q_new, q) <= r and q_new != q]
        for q in q_near:
            dubins = DubinsPath(q, q_new, self._r)
            dubins.calc_paths()
            path, length = dubins.get_shortest_path()
            xs, ys = DubinsPath.gen_path(q, path, self._r, section=False)
            if map_info.is_collision(path=[xs, ys]):
                continue
            if self.cost(q) + length < self.cost(q_new):
                self._rrt[q_new] = [q, path, [xs, ys]]
        for q in q_near:
            if q == self._rrt[q_new][0]:
                continue
            dubins = DubinsPath(q_new, q, self._r)
            dubins.calc_paths()
            path, length = dubins.get_shortest_path()
            xs, ys = DubinsPath.gen_path(q_new, path, self._r, section=False)
            if map_info.is_collision(path=[xs, ys]):
                continue
            if self.cost(q_new) + length < self.cost(q):
                self._rrt[q] = [q_new, path, [xs, ys]]

def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)

def rrt_star_dubins_planning(map_info, r, display=False):
    rrt = RRTDubins(map_info.start, r)
    while True:
        # generate random point
        if randint(0, 10) > 3:
            q_rand = (randint(1, map_info.width - 1), randint(1, map_info.height - 1), uniform(0, math.pi*2))
            if q_rand == map_info.start or (q_rand[0], q_rand[1]) in map_info.obstacle or rrt.is_contain(q_rand):
                continue
        else:
            q_rand = map_info.end
        pxy = rrt.extend(q_rand, map_info)
        if not pxy:
            continue
        rrt.rewire(q_rand, 15.0, map_info)
        if display:
            map_info.set_rand(q_rand)
            map_info.set_rrt_dubins(rrt.get_rrt())
        # check goal
        if q_rand == map_info.end:
            return reconstruct_path(rrt, map_info.end)

if __name__ == "__main__":
    m = MapInfo(60, 40)
    m.show()
    m.start = (10, 10, math.pi / 2)
    m.end = (50, 30, math.pi / 2)
    m.obstacle = [(20, i) for i in range(30)] + [(40, 40 - i) for i in range(30)]
    raw_input('enter to start ...')
    m.path = rrt_star_dubins_planning(m, 2.0, display=True)
    m.wait_close()