#include "theta_star.h"

ThetaStar::ThetaStar(MapInfo &map_info, bool display) : MotionPlanning(map_info, display)
{
    AStarPoint p;
    p.point.assign(MotionPlanning::_pt_start.begin(), MotionPlanning::_pt_start.end());
    p.g = 0.0;
    p.h = Distance(MotionPlanning::_pt_start, MotionPlanning::_pt_end);
    p.f = p.g + p.h;
    _openlist.push_back(p);
}

std::vector<KDPoint> ThetaStar::_NeighborPoints(KDPoint &p)
{
    double neighbor[8][2] = {{-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}};
    std::vector<KDPoint> points;
    for (int i = 0; i < 8; ++i)
    {
        KDPoint p_ = {p[0] + neighbor[i][0], p[1] + neighbor[i][1]};
        if (!MotionPlanning::_map_info.Collision(p_))
        {
            points.push_back(std::move(p_));
        }
    }
    return points;
}

std::vector<KDPoint> ThetaStar::_ReconstrucPath(void)
{
    std::vector<KDPoint> path;
    KDPoint p = MotionPlanning::_pt_end;
    while (p != MotionPlanning::_pt_start)
    {
        path.push_back(p);
        auto it = std::find_if(
            _closelist.begin(), _closelist.end(),
            [&](AStarPoint &pt)
            {
                return (p == pt.point);
            }
        );
        p = it->camefrom;
    }
    path.push_back(MotionPlanning::_pt_start);
    return path;
}

std::vector<KDPoint> ThetaStar::run(void)
{
    while (true)
    {
        auto it_min = std::min_element(
            _openlist.begin(), _openlist.end(),
            [](AStarPoint &p1, AStarPoint &p2)
            {
                return (p1.f < p2.f);
            }
        );
        AStarPoint x = *it_min;
        _openlist.erase(it_min);
        _closelist.push_back(x);
        if (x.point == MotionPlanning::_pt_end)
        {
            return _ReconstrucPath();
        }
        if (MotionPlanning::_display)
        {
            std::vector<KDPoint> list;
            for (auto o : _closelist)
                list.push_back(o.point);
            MotionPlanning::_map_info.set_closelist(list);
        }
        std::vector<KDPoint> neighbors = _NeighborPoints(x.point);
        for (auto y : neighbors)
        {
            // y is in close list
            if (std::find_if(_closelist.begin(), _closelist.end(), [&](AStarPoint &p){return (p.point == y);}) != _closelist.end())
                continue;
            double tentative_g = x.g + Distance(y, x.point);
            bool tentative_is_better = true;
            auto it_y = std::find_if(_openlist.begin(), _openlist.end(), [&](AStarPoint &p){return (p.point == y);});
            if (it_y == _openlist.end())
                tentative_is_better = true;
            else if (tentative_g < it_y->g)
                tentative_is_better = true;
            else
                tentative_is_better = false;
            if (tentative_is_better)
            {
                if ((x.point != MotionPlanning::_pt_start) && !MotionPlanning::_map_info.Collision(y, x.camefrom))
                {
                    if (it_y == _openlist.end())
                    {
                        AStarPoint y_;
                        y_.point.assign(y.begin(), y.end());
                        y_.g = tentative_g;
                        y_.h = Distance(y, MotionPlanning::_pt_end);
                        y_.f = y_.g + y_.h;
                        y_.camefrom.assign(x.camefrom.begin(), x.camefrom.end());
                        _openlist.push_back(y_);
                    }
                    else
                    {
                        it_y->g = tentative_g;
                        it_y->f = it_y->g + it_y->h;
                        it_y->camefrom.assign(x.camefrom.begin(), x.camefrom.end());
                    }
                }
                else
                {
                    if (it_y == _openlist.end())
                    {
                        AStarPoint y_;
                        y_.point.assign(y.begin(), y.end());
                        y_.g = tentative_g;
                        y_.h = Distance(y, MotionPlanning::_pt_end);
                        y_.f = y_.g + y_.h;
                        y_.camefrom.assign(x.point.begin(), x.point.end());
                        _openlist.push_back(y_);
                    }
                    else
                    {
                        it_y->g = tentative_g;
                        it_y->f = it_y->g + it_y->h;
                        it_y->camefrom.assign(x.point.begin(), x.point.end());
                    }
                }
                if (MotionPlanning::_display)
                {
                    std::vector<KDPoint> list;
                    for (auto o : _openlist)
                        list.push_back(o.point);
                    MotionPlanning::_map_info.set_openlist(list);
                }
            }
        }
    }
}
