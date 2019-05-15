#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from mapinfo import MapInfo
from random import randint
from scipy.spatial import cKDTree
import math
from rrt import RRT
from rrt import reconstruct_path as constructpath

def reconstruct_path(rrta, rrtb, q_reach, map_info):
    if rrta.is_root(map_info.start):
        rrt_start = rrta
        rrt_end = rrtb
    else:
        rrt_start = rrtb
        rrt_end = rrta
    p1 = constructpath(rrt_start, q_reach)
    p2 = constructpath(rrt_end, q_reach)
    p2.reverse()
    return p2 + p1

def rrt_connect_planning(map_info, display=False):
    rrt_start = RRT(map_info.start)
    rrt_end = RRT(map_info.end)
    rrt_a = rrt_start
    rrt_b = rrt_end
    okdtree = cKDTree(map_info.obstacle)
    while True:
        q_rand = (randint(1, map_info.width - 1), randint(1, map_info.height - 1))
        if q_rand == map_info.start or q_rand in map_info.obstacle or rrt_a.is_contain(q_rand):
            continue
        q_new = rrt_a.extend(q_rand, okdtree)
        if not q_new:
            continue
        while True:
            q_new_ = rrt_b.extend(q_new, okdtree)
            if display:
                map_info.set_rand(q_rand)
                map_info.set_rrt_connect(rrt_a.get_rrt(), rrt_b.get_rrt())
            # 2 rrts reached
            if q_new == q_new_:
                return reconstruct_path(rrt_a, rrt_b, q_new, map_info)
            elif q_new_ == None:
                break
        # swap 2 rrts
        rrt_a, rrt_b = rrt_b, rrt_a

if __name__ == "__main__":
    m = MapInfo(60, 40)
    m.show()
    m.start = (10, 10)
    m.end = (50, 30)
    m.obstacle = [(20, i) for i in range(30)] + [(40, 40 - i) for i in range(30)]
    raw_input('enter to start ...')
    m.path = rrt_connect_planning(m, display=True)
    m.wait_close()