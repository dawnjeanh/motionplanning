#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from mapinfo import MapInfo
from random import randint
from scipy.spatial import cKDTree
import math
from rrt import RRT as RRTBase
from rrt import reconstruct_path

class RRT(RRTBase):
    def __init__(self, q_init):
        super(RRT, self).__init__(q_init)

    def cost(self, q):
        q_now = q
        c = 0
        while(q_now != self._root):
            c += distance(q_now, self.get_parent(q_now))
            q_now = self.get_parent(q_now)
        return c

    def is_collision(self, p1, p2, okdtree):
        def middle_point(p1, p2):
            x1 = float(p1[0])
            y1 = float(p1[1])
            x2 = float(p2[0])
            y2 = float(p2[1])
            return ((x1+x2)/2, (y1+y2)/2)
        points = [p1, p2]
        L = distance(p1, p2)
        # generate points in line p1-p2, make sure number of points more than distance p1 to p2
        while len(points) < L*1.3:
            i = 0
            j = 1
            while j < len(points):
                points.insert(j, middle_point(points[i], points[j]))
                i += 2
                j += 2
        # judge each point collide obstacle or not
        for p in points:
            d, _ = okdtree.query(p)
            if d < 1.0:
                return True
        return False

    def rewire(self, q_new, r, okdtree):
        q_near = [q for q in self._rrt.keys() if distance(q_new, q) <= r]
        for q in q_near:
            if self.is_collision(q, q_new, okdtree):
                continue
            if self.cost(q) + distance(q, q_new) < self.cost(q_new):
                self._rrt[q_new] = q
        for q in q_near:
            if q == self._rrt[q_new]:
                continue
            if self.is_collision(q, q_new, okdtree):
                continue
            if self.cost(q_new) + distance(q, q_new) < self.cost(q):
                self._rrt[q] = q_new

def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def rrt_star_planning(map_info, display=False):
    rrt = RRT(map_info.start)
    okdtree = cKDTree(map_info.obstacle)
    while True:
        # generate random point
        if randint(0, 10) > 2:
            q_rand = (randint(1, map_info.width - 1), randint(1, map_info.height - 1))
            if q_rand == map_info.start or q_rand in map_info.obstacle or rrt.is_contain(q_rand):
                continue
        else:
            q_rand = map_info.end
        q_new = rrt.extend(q_rand, okdtree)
        if not q_new:
            continue
        rrt.rewire(q_new, 5.0, okdtree)
        if display:
            map_info.set_rand(q_rand)
            map_info.set_rrt(rrt.get_rrt())
        # check goal
        if distance(q_new, map_info.end) <= 1.0:
            if q_new != map_info.end:
                rrt.add(map_info.end, q_new)
            return reconstruct_path(rrt, map_info.end), rrt

if __name__ == "__main__":
    m = MapInfo(60, 40)
    m.show()
    m.start = (10, 10)
    m.end = (50, 30)
    m.obstacle = [(20, i) for i in range(30)] + [(40, 40 - i) for i in range(30)]
    raw_input('enter to start ...')
    m.path, _ = rrt_star_planning(m, display=True)
    m.wait_close()