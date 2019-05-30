#include "rrt_connect_plan.h"

RRTConnectPlan::RRTConnectPlan(MapInfo &map_info, bool display) : MotionPlanning(map_info, display)
{
    _rrt_start.set_root(MotionPlanning::_pt_start);
    _rrt_end.set_root(MotionPlanning::_pt_end);
}

KDPoint RRTConnectPlan::_GenerateRandPoint(void)
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

std::vector<KDPoint> RRTConnectPlan::_ReconstrucPath(RRT &rrt, KDPoint &start, KDPoint &end)
{
    std::vector<KDPoint> path;
    KDPoint p = end;
    while (p != start)
    {
        path.push_back(p);
        p = rrt.GetParent(p);
    }
    path.push_back(p);
    return path;
}

std::vector<KDPoint> RRTConnectPlan::_ReconstrucPath(KDPoint &link)
{
    std::vector<KDPoint> path1 = _ReconstrucPath(_rrt_start, MotionPlanning::_pt_start, link);
    std::vector<KDPoint> path2 = _ReconstrucPath(_rrt_end, MotionPlanning::_pt_end, link);
    path1.insert(path1.begin(), path2.rbegin(), path2.rend());
    return path1;
}

std::vector<KDPoint> RRTConnectPlan::run(void)
{
    RRT *rrt1 = &_rrt_start;
    RRT *rrt2 = &_rrt_end;
    while (true)
    {
        KDPoint q_rand = _GenerateRandPoint();
        KDPoint q_near1 = rrt1->SearchNearestVertex(q_rand);
        KDPoint q_new1 = rrt1->CalcNewPoint(q_near1, q_rand);
        if (MotionPlanning::_map_info.Collision(q_new1))
            continue;
        rrt1->Add(q_new1, q_near1);
        while (true)
        {
            KDPoint q_near2 = rrt2->SearchNearestVertex(q_new1);
            KDPoint q_new2 = rrt1->CalcNewPoint(q_near2, q_new1);
            if (MotionPlanning::_map_info.Collision(q_new2))
                break;
            rrt2->Add(q_new2, q_near2);
            if (q_new2 == q_new1)
            {
                if (MotionPlanning::_display)
                {
                    MotionPlanning::_map_info.set_rrt(_rrt_start, 0, q_rand);
                    MotionPlanning::_map_info.set_rrt(_rrt_end, 1, q_rand);
                }
                return _ReconstrucPath(q_new2);
            }
        }
        std::swap(rrt1, rrt2);
        if (MotionPlanning::_display)
        {
            MotionPlanning::_map_info.set_rrt(_rrt_start, 0, q_rand);
            MotionPlanning::_map_info.set_rrt(_rrt_end, 1, q_rand);
        }
    }
}