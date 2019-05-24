#include "prm.h"

void AddVertex(KDMap &kdmap, KDPoint &p1, KDPoint &p2)
{
    auto it_p = std::find_if(
        kdmap.begin(), kdmap.end(),
        [&](std::pair<KDPoint, std::vector<KDPoint>> &pair){return (pair.first == p1);}
    );
    if (it_p == kdmap.end())
    {
        std::vector<KDPoint> ps;
        ps.push_back(p2);
        kdmap.push_back(std::make_pair(p1, ps));
    }
    else
    {
        if (std::find_if(it_p->second.begin(), it_p->second.end(), [&](KDPoint &pt){return (pt == p2);}) == it_p->second.end())
            it_p->second.push_back(p2);
    }
}

std::vector<KDPoint> &GetAccess(KDMap &kdmap, KDPoint &p)
{
    auto it = std::find_if(
        kdmap.begin(), kdmap.end(),
        [&](std::pair<KDPoint, std::vector<KDPoint>> &pair)
        {
            return (pair.first == p);
        }
    );
    return it->second;
}

PRM::PRM(MapInfo &map_info, bool display) : MotionPlanning(map_info, display)
{
}

void PRM::_GenerateRandPoints(void)
{
    std::default_random_engine generator;
    std::uniform_int_distribution<int> dis_x(1, int(MotionPlanning::_map_info.get_width()) - 1);
    std::uniform_int_distribution<int> dis_y(1, int(MotionPlanning::_map_info.get_height()) - 1);
    while (_rand_points.size() < 300)
    {
        int x = dis_x(generator);
        int y = dis_y(generator);
        KDPoint p = {double(x), double(y)};
        if ((!MotionPlanning::_map_info.Collision(p)) &&
            (std::find_if(_rand_points.begin(), _rand_points.end(), [&](KDPoint &pt){return (pt == p);}) == _rand_points.end()) &&
            (p != MotionPlanning::_pt_start) && (p != MotionPlanning::_pt_end))
            _rand_points.push_back(std::move(p));
    }
}

void PRM::_GenerateRoadmap(void)
{
    KDTree kdt(_rand_points);
    for (auto p : _rand_points)
    {
        std::vector<std::pair<KDPoint, double>> result;
        kdt.Query(p, 7, result);
        std::vector<KDPoint> k_points;
        for (auto r : result)
        {
            if (p == r.first)
                continue;
            if (MotionPlanning::_map_info.Collision(p, r.first))
                continue;
            AddVertex(_road_map, p, r.first);
            AddVertex(_road_map, r.first, p);
        }
    }
}

std::vector<KDPoint> PRM::_ReconstrucPath(void)
{
    std::vector<KDPoint> path;
    KDPoint p = MotionPlanning::_pt_end;
    while (p != MotionPlanning::_pt_start)
    {
        path.push_back(p);
        auto it = std::find_if(
            _closelist.begin(), _closelist.end(),
            [&](DijkstraPoint &pt)
            {
                return (p == pt.point);
            }
        );
        p = it->camefrom;
    }
    path.push_back(MotionPlanning::_pt_start);
    return path;
}

std::vector<KDPoint> PRM::_DijkstraPlanning(void)
{
    DijkstraPoint p;
    p.point.assign(MotionPlanning::_pt_start.begin(), MotionPlanning::_pt_start.end());
    p.g = 0;
    _openlist.push_back(p);
    while (true)
    {
        // find x of min g in openlist
        auto it = std::min_element(
            _openlist.begin(), _openlist.end(),
            [](DijkstraPoint &p1, DijkstraPoint &p2)
            {
                return (p1.g < p2.g);
            }
        );
        DijkstraPoint x = *it;
        _openlist.erase(it);
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
        // find x neighbor
        std::vector<KDPoint> neighbor = GetAccess(_road_map, x.point);
        for (auto y : neighbor)
        {
            // y in close list
            if (std::find_if(_closelist.begin(), _closelist.end(), [&](DijkstraPoint &p){return (p.point == y);}) != _closelist.end())
                continue;
            double tentative_g = x.g + Distance(y, x.point);
            bool tentative_is_better = true;
            auto it_y = std::find_if(_openlist.begin(), _openlist.end(), [&](DijkstraPoint &p){return (p.point == y);});
            if (it_y == _openlist.end())
                tentative_is_better = true;
            else if (tentative_g < it_y->g)
                tentative_is_better = true;
            else
                tentative_is_better = false;
            if (tentative_is_better)
            {
                if (it_y == _openlist.end())
                {
                    DijkstraPoint y_;
                    y_.point.assign(y.begin(), y.end());
                    y_.g = tentative_g;
                    y_.camefrom.assign(x.point.begin(), x.point.end());
                    _openlist.push_back(y_);
                }
                else
                {
                    it_y->g = tentative_g;
                    it_y->camefrom.assign(x.point.begin(), x.point.end());
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

std::vector<KDPoint> PRM::run(void)
{
    _GenerateRandPoints();
    if (MotionPlanning::_display)
    {
        MotionPlanning::_map_info.set_rand_points(_rand_points);
        sleep(1);
    }
    _rand_points.push_back(MotionPlanning::_pt_start);
    _rand_points.push_back(MotionPlanning::_pt_end);
    _GenerateRoadmap();
    if (MotionPlanning::_display)
    {
        MotionPlanning::_map_info.set_roadmap(_road_map);
        sleep(1);
    }

    return _DijkstraPlanning();
}