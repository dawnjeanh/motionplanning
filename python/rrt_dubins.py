#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from mapinfo import MapInfo
from random import randint, uniform
import math
from dubins_path import DubinsPath

class RRTDubins(object):
    def __init__(self, q_init, r):
        self._root = q_init
        self._r = r
        self._rrt = {q_init:[q_init, [], []]}

    def search_nearest_vertex(self, p):
        rrt_list = self._rrt.items()
        distance = [(q[0][0] - p[0]) ** 2 + (q[0][1] - p[1]) ** 2 + (q[0][2] - p[2]) ** 2 for q in rrt_list]
        idx = distance.index(min(distance))
        return rrt_list[idx][0]

    def is_contain(self, q):
        return q in self._rrt

    def is_root(self, q):
        return q == self._root

    def add(self, q_new, q_near, path, xs, ys):
        self._rrt[q_new] = [q_near, path, [xs, ys]]

    def get_rrt(self):
        return self._rrt

    def get_info(self, q):
        return self._rrt[q]

    def extend(self, q_rand, map_info):
        # find nearest point in rrt
        q_near = self.search_nearest_vertex(q_rand)
        # calc dubins path
        dubins = DubinsPath(q_near, q_rand, self._r)
        dubins.calc_paths()
        path, _ = dubins.get_shortest_path()
        xs, ys = DubinsPath.gen_path(q_near, path, self._r, section=False)
        if map_info.is_collision(path=[xs, ys]):
            return []
        self.add(q_rand, q_near, path, xs, ys)
        return [xs, ys]

def reconstruct_path(rrt, end):
    path = []
    q = end
    while not rrt.is_root(q):
        q, _, pxy = rrt.get_info(q)
        path += list(reversed(zip(*pxy)))
    return list(reversed(path))

def rrt_dubins_planning(map_info, r, display=False):
    rrt = RRTDubins(map_info.start, r)
    
    while True:
        # generate random point
        if randint(0, 10) > 3:
            q_rand = (randint(1, map_info.width - 1), randint(1, map_info.height - 1), uniform(0, math.pi*2))
            if q_rand == map_info.start or (q_rand[0], q_rand[1]) in map_info.obstacle or rrt.is_contain(q_rand):
                continue
        else:
            q_rand = map_info.end
        dubins_path = rrt.extend(q_rand, map_info)
        if not dubins_path:
            continue
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
    m.path = rrt_dubins_planning(m, 2.0, display=True)
    m.wait_close()