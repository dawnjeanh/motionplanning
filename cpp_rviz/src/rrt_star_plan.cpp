#include "rrt_star_plan.h"

RRTStarPlan::RRTStarPlan(MapInfo &map_info, bool display) : MotionPlanning(map_info, display)
{
    _rrt.set_root(MotionPlanning::_pt_start);
}

KDPoint RRTStarPlan::_GenerateRandPoint(void)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_int_distribution<int> dis_s(0, 9);
    if (dis_s(generator) < 2)
    {
        return MotionPlanning::_pt_end;
        std::uniform_int_distribution<int> dis_s;
    }
    else
    {
        std::uniform_int_distribution<int> dis_x(1, int(MotionPlanning::_map_info.get_width()) - 1);
        std::uniform_int_distribution<int> dis_y(1, int(MotionPlanning::_map_info.get_height()) - 1);
        while (true)
        {
            int x = dis_x(generator);
            int y = dis_y(generator);
            KDPoint p = {double(x), double(y)};
            if (!MotionPlanning::_map_info.Collision(p))
            {
                return p;
            }
        }
    }
}

std::vector<KDPoint> RRTStarPlan::_ReconstrucPath(void)
{
    std::vector<KDPoint> path;
    KDPoint p = MotionPlanning::_pt_end;
    while (p != MotionPlanning::_pt_start)
    {
        path.push_back(p);
        p = _rrt.GetParent(p);
    }
    path.push_back(p);
    return path;
}

std::vector<KDPoint> RRTStarPlan::run(void)
{
    while (true)
    {
        KDPoint q_rand = _GenerateRandPoint();
        KDPoint q_near = _rrt.SearchNearestVertex(q_rand);
        KDPoint q_new = _rrt.CalcNewPoint(q_near, q_rand);
        if (MotionPlanning::_map_info.Collision(q_new))
            continue;
        _rrt.Add(q_new, q_near);
        _rrt.Rewire(q_new, 5.0, [&](KDPoint &p1, KDPoint &p2){return MotionPlanning::_map_info.Collision(p1, p2);});
        if (MotionPlanning::_display)
        {
            MotionPlanning::_map_info.set_rrt(_rrt, 0, q_rand);
        }
        if (Distance(q_new, MotionPlanning::_pt_end) < 1.0)
        {
            if (q_new != MotionPlanning::_pt_end)
                _rrt.Add(MotionPlanning::_pt_end, q_new);
            return _ReconstrucPath();
        }
    }
}