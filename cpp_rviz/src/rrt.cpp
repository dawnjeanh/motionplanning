#include "rrt.h"

void RRT::set_root(KDPoint &p)
{
    KDPoint r(p);
    _rrt.push_back(std::make_pair(r, 0));
}

KDPoint RRT::SearchNearestVertex(KDPoint &q_rand)
{
    std::vector<double> d;
    for (auto pair : _rrt)
    {
        d.push_back(Distance(pair.first, q_rand));
    }
    int i = std::min_element(d.begin(), d.end()) - d.begin();
    return _rrt[i].first;
}

KDPoint RRT::CalcNewPoint(KDPoint &q_near, KDPoint &q_rand)
{
    if (Distance(q_near, q_rand) < 1.0)
    {
        return q_rand;
    }
    double angle = std::atan2(q_rand[1] - q_near[1], q_rand[0] - q_near[0]);
    double x_new = q_near[0] + std::cos(angle);
    double y_new = q_near[1] + std::sin(angle);
    KDPoint p = {x_new, y_new};
    return p;
}

void RRT::Add(KDPoint &q_new, KDPoint &q_near)
{
    int i = std::find_if(
        _rrt.begin(), _rrt.end(),
        [&](std::pair<KDPoint, int> &pair)
        {
            return (pair.first == q_near);
        }
    ) - _rrt.begin();
    _rrt.push_back(std::make_pair(q_new, i));
}

KDPoint RRT::GetParent(KDPoint &p)
{
    auto it = std::find_if(
        _rrt.begin(), _rrt.end(),
        [&](std::pair<KDPoint, int> &pair)
        {
            return (pair.first == p);
        }
    );
    return _rrt[it->second].first;
}
