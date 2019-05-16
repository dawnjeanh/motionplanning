#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from mapinfo import MapInfo
from random import uniform, choice
from scipy.spatial import cKDTree
import math
from rrt_star import RRT, rrt_star_planning
from rrt import reconstruct_path, is_collision

def path_optimization(path, rrt, okdtree, map_info, display=False):
    path_opt = [path[0]]
    for i in range(1, len(path)):
        if rrt.is_collision(path_opt[-1], path[i], okdtree):
            path_opt.append(path[i-1])
    path_opt.append(path[-1])
    return path_opt

def sample(beacons, okdtree, r):
    rands = []
    for p in beacons:
        while True:
            rand = (uniform(p[0] - r/2.0, p[0] + r/2.0), uniform(p[1] - r/2.0, p[1] + r/2.0))
            if not is_collision(rand, okdtree):
                break
        rands.append(rand)
    return choice(rands)

def rrt_star_smart_planning(map_info, display=False):
    path, rrt = rrt_star_planning(map_info, display)
    if display:
        map_info.path = path
    okdtree = cKDTree(map_info.obstacle)
    path = path_optimization(path, rrt, okdtree, map_info, display)
    for _ in range(100):
        q_rand = sample(path[1:-1], okdtree, 5)
        q_new = rrt.extend(q_rand, okdtree)
        if not q_new:
            continue
        rrt.rewire(q_new, 5.0, okdtree)
        path = reconstruct_path(rrt, map_info.end)
        path = path_optimization(path, rrt, okdtree, map_info, display)
        if display:
            map_info.set_rand(q_rand)
            map_info.set_rrt(rrt.get_rrt())
            map_info.path = path
    path = path_optimization(path, rrt, okdtree, map_info, display)
    return path

if __name__ == "__main__":
    m = MapInfo(60, 40)
    m.show()
    m.start = (10, 10)
    m.end = (50, 30)
    m.obstacle = [(20, i) for i in range(30)] + [(40, 40 - i) for i in range(30)]
    raw_input('enter to start ...')
    m.path = rrt_star_smart_planning(m, display=True)
    m.wait_close()