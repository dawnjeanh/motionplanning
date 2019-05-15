#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import copy
import time

class MapInfo(object):
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self._border_x = [0, width, width, 0, 0]
        self._border_y = [0, 0, height, height, 0]
        self._start = (-1, -1)
        self._end = (-1, -1)
        self._obstacle = []
        self._open = (-1, -1)
        self._close = (-1, -1)
        self._path = []
        self._roadmap = dict()
        self._update_i = 0

    @property
    def start(self):
        return self._start

    @start.setter
    def start(self, s):
        self._start = s
        plt.plot(self.start[0], self.start[1], 'o', color='green')
        self.update()

    @property
    def end(self):
        return self._end

    @end.setter
    def end(self, e):
        self._end = e
        plt.plot(self.end[0], self.end[1], 'o', color='red')
        self.update()

    @property
    def obstacle(self):
        return self._obstacle

    @obstacle.setter
    def obstacle(self, o):
        self._obstacle = copy.deepcopy(o)
        t = zip(*self.obstacle)
        plt.plot(t[0], t[1], 's', color='black')
        self.update()

    @property
    def roadmap(self):
        return self._roadmap

    @roadmap.setter
    def roadmap(self, o):
        self._roadmap = copy.deepcopy(o)
        t = zip(*self.roadmap.keys())
        plt.plot(t[0], t[1], '.', color='blue')
        self.update()
        for k, v in self.roadmap.items():
            for p in v:
                plt.plot([k[0], p[0]], [k[1], p[1]], color='lightblue')
        plt.plot(t[0], t[1], '.', color='blue')
        self.update()

    def set_rand(self, r):
        plt.plot(r[0], r[1], 'd', color='blue')
        self.update()

    def set_rrt(self, rrt):
        plt.clf()
        plt.plot(self._border_x, self._border_y, 'black')
        plt.plot(self.start[0], self.start[1], 'o', color='green')
        plt.plot(self.end[0], self.end[1], 'o', color='red')
        t = zip(*self.obstacle)
        plt.plot(t[0], t[1], 's', color='black')
        for r in rrt.items():
            t = zip(*r)
            plt.plot(t[0], t[1], color='lightblue')
        if self._update_i % 20 == 1:
            self.update()

    def set_rrt_connect(self, rrta, rrtb):
        plt.clf()
        plt.plot(self._border_x, self._border_y, 'black')
        plt.plot(self.start[0], self.start[1], 'o', color='green')
        plt.plot(self.end[0], self.end[1], 'o', color='red')
        t = zip(*self.obstacle)
        plt.plot(t[0], t[1], 's', color='black')
        for r in rrta.items():
            t = zip(*r)
            plt.plot(t[0], t[1], color='lightblue')
        for r in rrtb.items():
            t = zip(*r)
            plt.plot(t[0], t[1], color='lightblue')
        if self._update_i % 20 == 1:
            self.update()

    @property
    def open(self):
        return self._open

    @open.setter
    def open(self, o):
        self._open = o
        plt.plot(self.open[0], self.open[1], 'x', color='lightblue')
        self._update_i += 1
        if self._update_i % 10 == 1:
            self.update()

    @property
    def close(self):
        return self._close

    @close.setter
    def close(self, o):
        self._close = o
        plt.plot(self.close[0], self.close[1], 'x', color='blue')
        self._update_i += 1
        if self._update_i % 10 == 1:
            self.update()

    @property
    def path(self):
        return self._path

    @path.setter
    def path(self, o):
        self._path = copy.deepcopy(o)
        t = zip(*self.path)
        plt.plot(t[0], t[1], color='purple')
        self.update()

    def show(self):
        plt.figure()
        plt.plot(self._border_x, self._border_y, 'black')
        plt.pause(0.001)

    def update(self):
        plt.pause(0.001)

    def wait_close(self):
        plt.show()


if __name__ == "__main__":
    m = MapInfo(60, 40)
    m.show()
    time.sleep(1)
    m.start = (10, 10)
    m.end = (50, 30)
    m.obstacle = [[(20, i) for i in range(30)]]
    m.wait_close()