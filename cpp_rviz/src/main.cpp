#include "map_info.h"
#include "kdtree.h"
#include "a_star.h"

int main(int argc, char** argv)
{
    MapInfo m(argc, argv);
    m.set_boundary(60, 40);

    std::vector<KDPoint> points;
    for (int i = 0; i < 30; ++i)
    {
        KDPoint p = {20, double(i)};
        points.push_back(p);
    }
    for (int i = 0; i < 30; ++i)
    {
        KDPoint p = {40, 40 - double(i)};
        points.push_back(p);
    }
    m.set_obstacle(points);

    KDPoint point = {10, 10};
    m.set_start(point);
    point = {50, 30};
    m.set_end(point);

    m.ShowMap();

    char c;
    std::cout << "press y to star:";
    std::cin >> c;

    AStar plan(m, true);
    std::vector<KDPoint> path = plan.run();

    m.set_path(path);

    std::cout << "end";
    std::cin >> c;

    return 0;
}