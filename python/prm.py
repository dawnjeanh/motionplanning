#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from mapinfo import MapInfo
from random import randint
from scipy.spatial import cKDTree
import math

def gen_sample_point(N, map_info):
    samples = []
    while len(samples) < N:
        x = randint(1, map_info.width - 1)
        y = randint(1, map_info.height - 1)
        if (x, y) not in map_info.obstacle and (x, y) not in samples:
            samples.append((x, y))
    return samples

def gen_roadmap(points, map_info):
    roadmap = dict()
    all_points = points + [map_info.start, map_info.end]
    pkdtree = cKDTree(all_points)
    okdtree = cKDTree(map_info.obstacle)
    i = 0
    for p in all_points:
        _, idx = pkdtree.query(p, k=10)
        for i in idx:
            if p == all_points[i]:
                continue
            if is_collision(p, all_points[i], okdtree):
                continue
            if p in roadmap:
                if all_points[i] not in roadmap[p]:
                    roadmap[p].append(all_points[i])
            else:
                roadmap[p] = [all_points[i]]
            if all_points[i] in roadmap:
                if p not in roadmap[all_points[i]]:
                    roadmap[all_points[i]].append(p)
            else:
                roadmap[all_points[i]] = [p]
    return roadmap

def is_collision(p1, p2, okdtree):
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

def middle_point(p1, p2):
    x1 = float(p1[0])
    y1 = float(p1[1])
    x2 = float(p2[0])
    y2 = float(p2[1])
    return ((x1+x2)/2, (y1+y2)/2)

def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def find_min_f(l, f_score):
    r = min(l, key=lambda item: f_score[item])
    i = l.index(r)
    return i

def reconstruct_path(camefrom, current_node):
    if current_node in camefrom:
        p = reconstruct_path(camefrom, camefrom[current_node])
        return p + [current_node]
    else:
        return [current_node]

def dijkstra_planning(roadmap, map_info):
    closedlist = []
    openlist = []
    camefrom = dict()
    g_score = {map_info.start: 0}
    openlist.append(map_info.start)
    while openlist:
        min_i = find_min_f(openlist, g_score)
        x = openlist.pop(min_i)
        if x == map_info.end:
            return reconstruct_path(camefrom, map_info.end)
        closedlist.append(x)
        for y in roadmap[x]:
            if y in closedlist:
                continue
            tentative_g_score = g_score[x] + distance(x, y)
            if y not in openlist:
                tentative_is_better = True
            elif tentative_g_score < g_score[y]:
                tentative_is_better = True
            else:
                tentative_is_better = False
            if tentative_is_better:
                camefrom[y] = x
                g_score[y] = tentative_g_score
                openlist.append(y)
    return []

def prm_planning(map_info, display=False):
    samples = gen_sample_point(300, map_info)
    roadmap = gen_roadmap(samples, map_info)
    if display:
        map_info.roadmap = roadmap
    path = dijkstra_planning(roadmap, map_info)
    return path

if __name__ == "__main__":
    m = MapInfo(60, 40)
    m.show()
    m.start = (10, 10)
    m.end = (50, 30)
    m.obstacle = [(20, i) for i in range(30)] + [(40, 40 - i) for i in range(30)]
    raw_input('enter to start ...')
    m.path = prm_planning(m, display=True)
    m.wait_close()
