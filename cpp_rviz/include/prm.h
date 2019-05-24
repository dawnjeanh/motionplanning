#ifndef __PRM__
#define __PRM__

#include <random>
#include <vector>
#include <tuple>
#include <utility>
#include "motionplanning.h"
#include "kdtree.h"

typedef std::vector<std::pair<KDPoint, std::vector<KDPoint>>> KDMap;
void AddVertex(KDMap &kdmap, KDPoint &p1, KDPoint &p2);
std::vector<KDPoint> &GetAccess(KDMap &kdmap, KDPoint &p);

struct DijkstraPoint
{
    KDPoint point;
    double g;
    KDPoint camefrom;
};

class PRM: public MotionPlanning
{
private:
    std::vector<KDPoint> _rand_points;
    KDMap _road_map;
    std::vector<DijkstraPoint> _openlist;
    std::vector<DijkstraPoint> _closelist;
    void _GenerateRandPoints(void);
    void _GenerateRoadmap(void);
    std::vector<KDPoint> _ReconstrucPath(void);
    std::vector<KDPoint> _DijkstraPlanning(void);
public:
    PRM(MapInfo &map_info, bool display);
    std::vector<KDPoint> run(void);
};




#endif // !__PRM__
