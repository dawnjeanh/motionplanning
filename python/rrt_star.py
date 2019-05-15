#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from mapinfo import MapInfo
from random import randint
from scipy.spatial import cKDTree
import math
from rrt import RRT

def rrt_star_planning(map_info, display=False):
    rrt = RRT(map_info.start)

if __name__ == "__main__":
    m = MapInfo(60, 40)
    m.show()
    m.start = (10, 10)
    m.end = (50, 30)
    m.obstacle = [(20, i) for i in range(30)] + [(40, 40 - i) for i in range(30)]
    rrt_star_planning(m, display=True)
    m.wait_close()