#ifndef __RRT_PLAN__
#define __RRT_PLAN__

#include <vector>
#include <random>
#include <chrono>
#include "motionplanning.h"
#include "kdtree.h"
#include "rrt.h"

class RRTPlan : public MotionPlanning
{
private:
    RRT _rrt;
    KDPoint _GenerateRandPoint(void);
    std::vector<KDPoint> _ReconstrucPath(void);
public:
    RRTPlan(MapInfo &map_info, bool display);
    std::vector<KDPoint> run(void);
};

#endif // !__RRT_PLAN__