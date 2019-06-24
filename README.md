# motionplanning
Motion planning algorithm implementation in Python and C++

# Table of Contents
    * [C++ useage](#c++-useage)
    * [a* algorithm](#a*-algorithm)
    * [theta* algorithm](#theta*-algorithm)
    * [Probabilistic Roadmaps algorithm](#probabilistic-roadmaps-algorithm)
    * [Rapidly-exploring random tree algorithm](#rapidly-exploring-random-tree-algorithm)
        * [rrt](#rrt)
        * [rrt_connect](#rrt_connect)
        * [rrt*](#rrt*)
        * [rrt*-smart](#rrt*-smart)
    * [Dubins path algorithm](#dubins-path-algorithm)
        * [shortest dubins path with different end point](#shortest-dubins-path-with-different-end-point)
        * [6 types of dubins path with same end point](#6-types-of-dubins-path-with-same-end-point)
        * [RRT-Dubins](#rrt-dubins)
        * [RRT*-Dubins](#rrt*-dubins)

## C++ useage

C++11 standard, use Rviz to show algorithm, so you should install [ROS](https://www.ros.org/).

1. put this package in your ros workspace, e.g. ~/catkin_ws/src/
2. cd ~/catkin_ws and catkin_make
3. source devel/setup.bash
4. launch script, e.g. roslaunch cpp_rviz a_star.launch

## a* algorithm
![a_star](https://github.com/dawnjeanh/image_resource/raw/master/gif/a_star_rviz.gif)

## theta* algorithm
![theta_star](https://github.com/dawnjeanh/image_resource/raw/master/gif/theta_star_rviz.gif)

## Probabilistic Roadmaps algorithm
![prm](https://github.com/dawnjeanh/image_resource/raw/master/gif/prm_rviz.gif)

## Rapidly-exploring random tree algorithm

### rrt
![rrt](https://github.com/dawnjeanh/image_resource/raw/master/gif/rrt_rviz.gif)

### rrt_connect
![rrt_connect](https://github.com/dawnjeanh/image_resource/raw/master/gif/rrt_connect_rviz.gif)

### rrt*
![rrt_connect](https://github.com/dawnjeanh/image_resource/raw/master/gif/rrt_star_rviz.gif)

### rrt*-smart
![rrt_connect](https://github.com/dawnjeanh/image_resource/raw/master/gif/rrt_star_smart.gif)

## Dubins path algorithm
[Dubins](https://github.com/dawnjeanh/motionplanning/tree/master/doc/Dubins_path.md) algorithm have 6 types path as below.
### shortest dubins path with different end point
![](https://github.com/dawnjeanh/image_resource/raw/master/png/Dubins_1.png)
### 6 types of dubins path with same end point
![](https://github.com/dawnjeanh/image_resource/raw/master/png/Dubins_2.png)
### RRT-Dubins
![rrt_dubins](https://github.com/dawnjeanh/image_resource/raw/master/gif/rrt_dubins.gif)
### RRT*-Dubins
![rrt_star_dubins](https://github.com/dawnjeanh/image_resource/raw/master/gif/rrt_star_dubins.gif)