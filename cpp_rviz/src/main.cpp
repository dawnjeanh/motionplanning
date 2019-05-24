#include "map_info.h"
#include "kdtree.h"
#include "a_star.h"
#include "theta_star.h"
#include "prm.h"

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

    std::string type;
    ros::param::get("/type", type);
    std::vector<KDPoint> path;
    if (type.compare("a_star") == 0)
    {
        AStar plan(m, true);
        path = plan.run();
    }
    else if (type.compare("theta_star") == 0)
    {
        ThetaStar plan(m, true);
        path = plan.run();
    }
    else if (type.compare("prm") == 0)
    {
        PRM plan(m, true);
        path = plan.run();
    }
    else
    {
        std::cout << type << " is not exist" << std::endl;
    }

    if (!path.empty())
        m.set_path(path);

    std::cout << "end";
    std::cin >> c;

    return 0;
}