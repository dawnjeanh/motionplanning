#ifndef __RRT_STAR_PLAN__
#define __RRT_STAR_PLAN__

#include <vector>
#include <random>
#include <chrono>
#include "motionplanning.h"
#include "kdtree.h"
#include "rrt.h"

class RRTStarPlan : public MotionPlanning
{
private:
    RRT _rrt;
    KDPoint _GenerateRandPoint(void);
    std::vector<KDPoint> _ReconstrucPath(void);
public:
    RRTStarPlan(MapInfo &map_info, bool display);
    std::vector<KDPoint> run(void);
};

#endif // !__RRT_STAR_PLAN__