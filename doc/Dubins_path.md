# Dubins Path note

I have studied Dubins path algorithm, and make the record.

There are 6 types of Dubins path, LSL, RSR, LSR, RSL, LRL, RLR. R is short for Right, L is short for Left, S is short for Straight.

In order to facilitate the calculation, we transform the coordinates of start point and end point. Start point transform to (0, 0) and yaw of start point is 0(rad), end point transform to (x, y) and yaw of end point is phi(rad).

LSL and RSR are mirror images of each other, we can calculate RSR path of end point (x, y, phi) through calculate LSL path of end point (x, -y, -phi), so dose LSR and RSL, LRL and RLR.

So LSL, LSR, LRL solution as below.

![LSL](https://github.com/dawnjeanh/motionplanning/raw/master/doc/Dubins_LSL.png)

![LSR](https://github.com/dawnjeanh/motionplanning/raw/master/doc/Dubins_LSR.png)

![LRL](https://github.com/dawnjeanh/motionplanning/raw/master/doc/Dubins_LRL.png)