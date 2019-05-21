#ifndef __MOTIONPLANNING__
#define __MOTIONPLANNING__

#include "map_info.h"
#include <vector>

class MotionPlanning
{
public:
    MapInfo &_map_info;
    bool _display;
    KDPoint _pt_start;
    KDPoint _pt_end;
    MotionPlanning(MapInfo &map_info, bool display): _map_info(map_info)
    {
        _display = display;
        _pt_start.assign(map_info.pt_start.begin(), map_info.pt_start.end());
        _pt_end.assign(map_info.pt_end.begin(), map_info.pt_end.end());
    }
    virtual std::vector<KDPoint> run(void) = 0;
};

#endif // !__MOTIONPLANNING__
