#ifndef __THETA_STAR__
#define __THETA_STAR__

#include "motionplanning.h"
#include "kdtree.h"
#include "a_star.h"
#include <vector>


class ThetaStar: public MotionPlanning
{
private:
    std::vector<AStarPoint> _openlist;
    std::vector<AStarPoint> _closelist;
    std::vector<std::pair<KDPoint, KDPoint>> _came_from;
    std::vector<KDPoint> _ReconstrucPath(void);
    std::vector<KDPoint> _NeighborPoints(KDPoint &p);
public:
    ThetaStar(MapInfo &map_info, bool display);
    std::vector<KDPoint> run(void);
};

#endif // !__THETA_STAR__