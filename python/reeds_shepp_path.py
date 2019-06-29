#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
from math import sin, cos, asin, acos, atan2, sqrt, pi, floor
from numpy import arange, mod
from copy import deepcopy

def draw_point(point, arrow_length=0.5):
    plt.plot(point[0], point[1], 'o')
    plt.arrow(point[0], point[1], arrow_length * cos(point[2]), arrow_length * sin(point[2]), head_width=0.05)

class ReedsSheppPath(object):
    def __init__(self, start, end, r=1.0):
        self._s = start
        self._e = end
        self._r = r
        self._paths = []

    def calc_paths(self):
        x, y, phi = self.calc_end()
        self._paths += self.CSC(x, y, phi)
        self._paths += self.CCC(x, y, phi)
        self._paths += self.CCCC(x, y, phi)
        self._paths += self.CCSC(x, y, phi)
        self._paths += self.CCSCC(x, y, phi)
        return self._paths

    def get_shortest_path(self):
        shortest_cost = float("inf")
        shortest_path = []
        for path in self._paths:
            cost = 0
            for p in path:
                cost += abs(p[1]) if p[0] == 's' else abs(p[1]) * self._r
            if cost < shortest_cost:
                shortest_path = path
                shortest_cost = cost
        return deepcopy(shortest_path), shortest_cost

    def calc_end(self):
        ex = self._e[0] - self._s[0]
        ey = self._e[1] - self._s[1]

        lex = cos(self._s[2]) * ex + sin(self._s[2]) * ey
        ley = - sin(self._s[2]) * ex + cos(self._s[2]) * ey
        leyaw = self._e[2] - self._s[2]
        lex = lex / self._r
        ley = ley / self._r
        return [lex, ley, leyaw]

    def polar(self, x, y):
        r = sqrt(x ** 2 + y ** 2)
        theta = atan2(y, x)
        return r, theta

    def mod2pi(self, theta):
        v = mod(theta, 2.0 * pi)
        if v < -pi:
            v += 2.0 * pi
        else:
            if v > pi:
                v -= 2.0 * pi
        return v

    def tau_omega(self, u, v, xi, eta, phi):
        delta = self.mod2pi(u - v)
        A = sin(u) - sin(delta)
        B = cos(u) - cos(delta) - 1
        _, t1 = self.polar(xi * A + eta * B, eta * A - xi * B)
        t2 = 2 * cos(delta) - 2 * cos(v) - 2 * cos(u) + 3
        if t2 < 0:
            tau = self.mod2pi(t1 + pi)
        else:
            tau = self.mod2pi(t1)
        omega = self.mod2pi(tau - u + v - phi)
        return tau, omega

    def LpSpLp(self, x, y, phi):
        " formula 8.1 "
        u, t = self.polar(x - sin(phi), y - 1 + cos(phi))
        if t > 0:
            v = self.mod2pi(phi - t)
            if v > 0:
                return [['l', t], ['s', u * self._r], ['l', v]]
        return []

    def LmSmLm(self, x, y, phi):
        " timeflip of LpSpLp "
        path = self.LpSpLp(-x, y, -phi)
        if path:
            d, [t, u, v] = zip(*path)
            return zip(d, [-t, -u, -v])
        return []

    def RpSpRp(self, x, y, phi):
        " reflect of LpSpLp "
        path = self.LpSpLp(x, -y, -phi)
        if path:
            _, [t, u, v] = zip(*path)
            return zip(['r', 's', 'r'], [t, u, v])
        return []

    def RmSmRm(self, x, y, phi):
        " timeflp + reflect of LpSpLp "
        path = self.LpSpLp(-x, -y, phi)
        if path:
            _, [t, u, v] = zip(*path)
            return zip(['r', 's', 'r'], [-t, -u, -v])
        return []

    def LpSpRp(self, x, y, phi):
        " formula 8.2 "
        u1, t1 = self.polar(x + sin(phi), y - 1 - cos(phi))
        if u1 ** 2 >= 4.0:
            u = sqrt(u1 ** 2 - 4)
            _, theta = self.polar(u, 2)
            t = self.mod2pi(t1 + theta)
            v = self.mod2pi(t - phi)
            if t > 0 and v > 0:
                return [['l', t], ['s', u * self._r], ['r', v]]
        return []

    def LmSmRm(self, x, y, phi):
        " timeflip of LpSpRp "
        path = self.LpSpRp(-x, y, -phi)
        if path:
            d, [t, u, v] = zip(*path)
            return zip(d, [-t, -u, -v])
        return []

    def RpSpLp(self, x, y, phi):
        " reflect of LpSpRp "
        path = self.LpSpRp(x, -y, -phi)
        if path:
            _, [t, u, v] = zip(*path)
            return zip(['r', 's', 'l'], [t, u, v])
        return []

    def RmSmLm(self, x, y, phi):
        " timeflp + reflect of LpSpRp "
        path = self.LpSpRp(-x, -y, phi)
        if path:
            _, [t, u, v] = zip(*path)
            return zip(['r', 's', 'l'], [-t, -u, -v])
        return []

    def CSC(self, x, y, phi):
        paths = []
        planners = [self.LpSpLp, self.LmSmLm, self.RpSpRp, self.RmSmRm,
                    self.LpSpRp, self.LmSmRm, self.RpSpLp, self.RmSmLm]
        for planner in planners:
            path = planner(x, y, phi)
            if path:
                paths.append(path)
        return paths

    def LpRmL(self, x, y, phi):
        " formula 8.3 & 8.4 "
        xi = x - sin(phi)
        eta = y - 1 + cos(phi)
        u1, theta = self.polar(xi, eta)
        if u1 <= 4.0:
            u = -2 * asin(u1 / 4)
            t = self.mod2pi(theta + u / 2 + pi)
            v = self.mod2pi(phi - t + u)
            if t >= 0 and u <= 0:
                return [['l', t], ['r', u], ['l', v]]
        return []

    def LmRpL(self, x, y, phi):
        " timeflip of LpRmL "
        path = self.LpRmL(-x, y, -phi)
        if path:
            d, [t, u, v] = zip(*path)
            return zip(d, [-t, -u, -v])
        return []

    def RpLmR(self, x, y, phi):
        " reflect of LpRmL "
        path = self.LpRmL(x, -y, -phi)
        if path:
            _, [t, u, v] = zip(*path)
            return zip(['r', 'l', 'r'], [t, u, v])
        return []

    def RmLpR(self, x, y, phi):
        " timeflp + reflect of LpRmL "
        path = self.LpRmL(-x, -y, phi)
        if path:
            _, [t, u, v] = zip(*path)
            return zip(['r', 'l', 'r'], [-t, -u, -v])
        return []

    def CCC(self, x, y, phi):
        paths = []
        planners = [self.LpRmL, self.LmRpL, self.RpLmR, self.RmLpR]
        for planner in planners:
            path = planner(x, y, phi)
            if path:
                paths.append(path)
        # backwards
        xb = x * cos(phi) + y * sin(phi)
        yb = x * sin(phi) - y * cos(phi)
        for planner in planners:
            path = planner(xb, yb, phi)
            if path:
                d, [t, u, v] = zip(*path)
                path = zip(d, [v, u, t])
                paths.append(path)
        return paths

    def LpRpLmRm(self, x, y, phi):
        " formula 8.7 "
        xi = x + sin(phi)
        eta = y - 1 - cos(phi)
        rho = (2 + sqrt(xi ** 2 + eta ** 2)) / 4
        if rho <= 1.0:
            u = acos(rho)
            t, v = self.tau_omega(u, -u, xi, eta, phi)
            if t >= 0 and v <= 0:
                return [['l', t], ['r', u], ['l', -u], ['r', v]]
        return []

    def LmRmLpRp(self, x, y, phi):
        " timeflip of LpRpLmRm "
        path = self.LpRpLmRm(-x, y, -phi)
        if path:
            d, [t, u, u1, v] = zip(*path)
            return zip(d, [-t, -u, -u1, -v])
        return []

    def RpLpRmLm(self, x, y, phi):
        " reflect of LpRpLmRm "
        path = self.LpRpLmRm(x, -y, -phi)
        if path:
            _, [t, u, u1, v] = zip(*path)
            return zip(['r', 'l', 'r', 'l'], [t, u, u1, v])
        return []

    def RmLmRpLp(self, x, y, phi):
        " timeflp + reflect of LpRpLmRm "
        path = self.LpRpLmRm(-x, -y, phi)
        if path:
            _, [t, u, u1, v] = zip(*path)
            return zip(['r', 'l', 'r', 'l'], [-t, -u, -u1, -v])
        return []

    def LpRmLmRp(self, x, y, phi):
        " formula 8.8 "
        xi = x + sin(phi)
        eta = y - 1 - cos(phi)
        rho = (20 - xi ** 2 - eta ** 2) / 16
        if 0.0 <= rho <= 1.0:
            u = -acos(rho)
            t, v = self.tau_omega(u, u, xi, eta, phi)
            if t >= 0 and v >= 0:
                return [['l', t], ['r', u], ['l', u], ['r', v]]
        return []

    def LmRpLpRm(self, x, y, phi):
        " timeflip of LpRmLmRp "
        path = self.LpRmLmRp(-x, y, -phi)
        if path:
            d, [t, u, u1, v] = zip(*path)
            return zip(d, [-t, -u, -u1, -v])
        return []

    def RpLmRmLp(self, x, y, phi):
        " reflect of LpRmLmRp "
        path = self.LpRmLmRp(x, -y, -phi)
        if path:
            _, [t, u, u1, v] = zip(*path)
            return zip(['r', 'l', 'r', 'l'], [t, u, u1, v])
        return []

    def RmLpRpLm(self, x, y, phi):
        " timeflp + reflect of LpRmLmRp "
        path = self.LpRmLmRp(-x, -y, phi)
        if path:
            _, [t, u, u1, v] = zip(*path)
            return zip(['r', 'l', 'r', 'l'], [-t, -u, -u1, -v])
        return []

    def CCCC(self, x, y, phi):
        paths = []
        planners = [self.LpRpLmRm, self.LmRmLpRp, self.RpLpRmLm, self.RmLmRpLp,
                    self.LpRmLmRp, self.LmRpLpRm, self.RpLmRmLp, self.RmLpRpLm]
        for planner in planners:
            path = planner(x, y, phi)
            if path:
                paths.append(path)
        return paths

    def LpRmSmLm(self, x, y, phi):
        " formula 8.9 "
        xi = x - sin(phi)
        eta = y - 1 + cos(phi)
        rho, theta = self.polar(xi, eta)
        if rho >= 2.0:
            r = sqrt(rho**2 - 4)
            u = 2 - r
            t = self.mod2pi(theta + atan2(r, -2))
            v = self.mod2pi(phi - pi / 2 - t)
            if t >= 0.0 and u <= 0.0 and v <= 0.0:
                return [['l', t], ['r', -pi / 2], ['s', u * self._r], ['l', v]]
        return []

    def LmRpSpLp(self, x, y, phi):
        " timeflip of LpRmSmLm "
        path = self.LpRmSmLm(-x, y, -phi)
        if path:
            d, [t, u, u1, v] = zip(*path)
            return zip(d, [-t, -u, -u1, -v])
        return []

    def RpLmSmRm(self, x, y, phi):
        " reflect of LpRmSmLm "
        path = self.LpRmSmLm(x, -y, -phi)
        if path:
            _, [t, u, u1, v] = zip(*path)
            return zip(['r', 'l', 's', 'r'], [t, u, u1, v])
        return []

    def RmLpSpRp(self, x, y, phi):
        " timeflp + reflect of LpRmSmLm "
        path = self.LpRmSmLm(-x, -y, phi)
        if path:
            _, [t, u, u1, v] = zip(*path)
            return zip(['r', 'l', 's', 'r'], [-t, -u, -u1, -v])
        return []

    def LpRmSmRm(self, x, y, phi):
        " formula 8.10 "
        xi = x + sin(phi)
        eta = y - 1 - cos(phi)
        rho, theta = self.polar(-eta, xi)
        if rho >= 2.0:
            t = theta
            u = 2 - rho
            v = self.mod2pi(t + pi / 2 - phi)
            if t >= 0.0 and u <= 0.0 and v <= 0.0:
                return [['l', t], ['r', -pi / 2], ['s', u * self._r], ['r', v]]
        return []

    def LmRpSpRp(self, x, y, phi):
        " timeflip of LpRmSmRm "
        path = self.LpRmSmRm(-x, y, -phi)
        if path:
            d, [t, u, u1, v] = zip(*path)
            return zip(d, [-t, -u, -u1, -v])
        return []

    def RpLmSmLm(self, x, y, phi):
        " reflect of LpRmSmRm "
        path = self.LpRmSmRm(x, -y, -phi)
        if path:
            _, [t, u, u1, v] = zip(*path)
            return zip(['r', 'l', 's', 'l'], [t, u, u1, v])
        return []

    def RmLpSpLp(self, x, y, phi):
        " timeflp + reflect of LpRmSmRm "
        path = self.LpRmSmRm(-x, -y, phi)
        if path:
            _, [t, u, u1, v] = zip(*path)
            return zip(['r', 'l', 's', 'l'], [-t, -u, -u1, -v])
        return []

    def CCSC(self, x, y, phi):
        paths = []
        planners = [self.LpRmSmLm, self.LmRpSpLp, self.RpLmSmRm, self.RmLpSpRp,
                    self.LpRmSmRm, self.LmRpSpRp, self.RpLmSmLm, self.RmLpSpLp]
        for planner in planners:
            path = planner(x, y, phi)
            if path:
                paths.append(path)
        # backwards
        xb = x * cos(phi) + y * sin(phi)
        yb = x * sin(phi) - y * cos(phi)
        for planner in planners:
            path = planner(xb, yb, phi)
            if path:
                [d1, d2, d3, d4], [t, u, u1, v] = zip(*path)
                path = zip([d4, d3, d2, d1], [v, u1, u, t])
                paths.append(path)
        return paths

    def LpRmSmLmRp(self, x, y, phi):
        " formula 8.11 "
        xi = x + sin(phi)
        eta = y - 1 - cos(phi)
        rho, _ = self.polar(xi, eta)
        if rho >= 2.0:
            u = 4 - sqrt(rho ** 2 - 4)
            if u <= 0.0:
                t = self.mod2pi(atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta))
                v = self.mod2pi(t - phi)
                if u <= 0.0 and v >= 0.0:
                    return [['l', t], ['r', -pi/2], ['s', u * self._r], ['l', -pi/2], ['r', v]]
        return []

    def LmRpSpLpRm(self, x, y, phi):
        " timeflip of LpRmSmLmRp "
        path = self.LpRmSmLmRp(-x, y, -phi)
        if path:
            d, [t, u, u1, u2, v] = zip(*path)
            return zip(d, [-t, -u, -u1, -u2, -v])
        return []

    def RmLpSpRpLm(self, x, y, phi):
        " reflect of LpRmSmLmRp "
        path = self.LpRmSmLmRp(x, -y, -phi)
        if path:
            _, [t, u, u1, u2, v] = zip(*path)
            return zip(['r', 'l', 's', 'r', 'l'], [t, u, u1, u2, v])
        return []

    def RpLmSmRmLp(self, x, y, phi):
        " timeflp + reflect of LpRmSmLmRp "
        path = self.LpRmSmLmRp(-x, -y, phi)
        if path:
            _, [t, u, u1, u2, v] = zip(*path)
            return zip(['r', 'l', 's', 'r', 'l'], [-t, -u, -u1, -u2, -v])
        return []

    def CCSCC(self, x, y, phi):
        paths = []
        planners = [self.LpRmSmLmRp, self.LmRpSpLpRm, self.RmLpSpRpLm, self.RpLmSmRmLp]
        for planner in planners:
            path = planner(x, y, phi)
            if path:
                paths.append(path)
        return paths

    @classmethod
    def gen_path(cls, s, path, r=1.0, section=True):
        def calc_TurnCenter(point, dir='l', r=1.0):
            if dir == 'l':
                ang = point[2] + pi / 2
            elif dir == 'r':
                ang = point[2] - pi / 2
            else:
                return None
            x = point[0] + cos(ang) * r
            y = point[1] + sin(ang) * r
            return (x, y)
        r_x = []
        r_y = []
        ps_x = []
        ps_y = []
        start = s
        yaw = s[2]
        for p in path:
            if p[0] == 's':
                for l in arange(0, p[1], 0.5 if p[1] > 0 else -0.5):
                    ps_x.append(start[0] + cos(yaw) * l)
                    ps_y.append(start[1] + sin(yaw) * l)
                ps_x.append(start[0] + cos(yaw) * p[1])
                ps_y.append(start[1] + sin(yaw) * p[1])
                if section:
                    r_x.append(ps_x)
                    r_y.append(ps_y)
                else:
                    r_x += ps_x
                    r_y += ps_y
            else:
                center = calc_TurnCenter(start, p[0], r)
                ang_start = atan2(start[1] - center[1], start[0] - center[0])
                ang_end = ang_start + p[1] if p[0] == 'l' else ang_start - p[1]
                step = 0.1 / r
                for ang in arange(ang_start, ang_end, step if ang_start < ang_end else -step):
                    ps_x.append(center[0] + cos(ang) * r)
                    ps_y.append(center[1] + sin(ang) * r)
                ps_x.append(center[0] + cos(ang_end) * r)
                ps_y.append(center[1] + sin(ang_end) * r)
                if section:
                    r_x.append(ps_x)
                    r_y.append(ps_y)
                else:
                    r_x += ps_x
                    r_y += ps_y
                yaw = start[2] + p[1] if p[0] == 'l' else start[2] - p[1]
            start = (ps_x[-1], ps_y[-1], yaw)
            ps_x = []
            ps_y = []
        return r_x, r_y

def test1():
    for i in range(0, 360, 30):
        plt.figure()
        start = [1, 1, pi / 30]
        end = [3, 4, i / 180.0 * pi]
        r = 1.0
        rspath = ReedsSheppPath(start, end, r)
        paths = rspath.calc_paths()
        print paths
        for i, path in enumerate(paths):
            plt.subplot(3, 4, i+1)
            title = ''
            for p in path:
                title += p[0] + ('+' if p[1] > 0 else '-')
            plt.title(title)
            draw_point(start)
            draw_point(end)
            xs, ys = ReedsSheppPath.gen_path(start, path, r)
            for i in range(len(xs)):
                plt.plot(xs[i], ys[i])
            plt.axis("equal")
        plt.show()

def test2():
    plt.figure()
    for i in range(12):
        start = [1, 1, 30 / 180.0 * pi]
        end = [3, 4, i * 30 / 180.0 * pi]
        r = 1.0
        rspath = ReedsSheppPath(start, end, r)
        rspath.calc_paths()
        path, _ = rspath.get_shortest_path()
        print path
        plt.subplot(3, 4, i+1)
        title = ''
        for p in path:
            title += p[0] + ('+' if p[1] > 0 else '-')
        plt.title(title)
        draw_point(start)
        draw_point(end)
        xs, ys = ReedsSheppPath.gen_path(start, path, r)
        for i in range(len(xs)):
            plt.plot(xs[i], ys[i])
        plt.axis("equal")
    plt.show()

if __name__ == "__main__":
    # test1()
    test2()