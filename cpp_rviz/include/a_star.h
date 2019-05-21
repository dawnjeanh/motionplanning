#ifndef __A_STAR__
#define __A_STAR__

#include "motionplanning.h"
#include "kdtree.h"
#include <vector>

struct AStarPoint
{
    KDPoint point;
    double g;
    double h;
    double f;
    KDPoint camefrom;
};

class AStar: public MotionPlanning
{
private:
    std::vector<AStarPoint> _openlist;
    std::vector<AStarPoint> _closelist;
    std::vector<std::pair<KDPoint, KDPoint>> _came_from;
    std::vector<KDPoint> _ReconstrucPath(void);
    std::vector<KDPoint> _NeighborPoints(KDPoint &p);
public:
    AStar(MapInfo &map_info, bool display);
    std::vector<KDPoint> run(void);
};

#endif // !__A_STAR__