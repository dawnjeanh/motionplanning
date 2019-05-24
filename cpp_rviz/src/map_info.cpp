#include "map_info.h"

MapInfo::MapInfo(int argc, char** argv)
{
    ros::init(argc, argv, "map");
    ros::NodeHandle n;
    _marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 50);
    _pub_i = 0;
}

MapInfo::~MapInfo()
{
    _marker_pub.shutdown();
}

void MapInfo::set_boundary(int w, int h)
{
    _width = double(w);
    _height = double(h);

    _line_boundary.header.frame_id = "/my_frame";
    _line_boundary.header.stamp = ros::Time::now();
    _line_boundary.action = visualization_msgs::Marker::ADD;
    _line_boundary.ns = "map";
    _line_boundary.id = _id_boundary;
    _line_boundary.type = visualization_msgs::Marker::LINE_STRIP;
    _line_boundary.pose.orientation.w = 1.0;
    _line_boundary.scale.x = 1;
    _line_boundary.color.a = 1.0;

    _line_boundary.points.clear();
    geometry_msgs::Point p;
    p.x = 0.0;
    p.y = 0.0;
    _line_boundary.points.push_back(p);
    p.x = _width;
    p.y = 0.0;
    _line_boundary.points.push_back(p);
    p.x = _width;
    p.y = _height;
    _line_boundary.points.push_back(p);
    p.x = 0.0;
    p.y = _height;
    _line_boundary.points.push_back(p);
    p.x = 0.0;
    p.y = 0.0;
    _line_boundary.points.push_back(p);
}

void MapInfo::set_obstacle(std::vector<KDPoint> &points)
{
    _okdtree = KDTree(points);
    // _okdtree.PrintKDTree();

    _obstacle.header.frame_id = "/my_frame";
    _obstacle.header.stamp = ros::Time::now();
    _obstacle.action = visualization_msgs::Marker::ADD;
    _obstacle.ns = "map";
    _obstacle.id = _id_obstacle;
    _obstacle.type = visualization_msgs::Marker::POINTS;
    _obstacle.pose.orientation.w = 1.0;
    _obstacle.scale.x = 1;
    _obstacle.color.a = 1.0;

    _obstacle.points.clear();
    for (auto p : points)
    {
        geometry_msgs::Point p_;
        p_.x = p[0];
        p_.y = p[1];
        p_.z = 0;
        _obstacle.points.push_back(p_);
    }
}

void MapInfo::set_start(KDPoint &point)
{
    pt_start.assign(point.begin(), point.end());

    _m_start.header.frame_id = "/my_frame";
    _m_start.header.stamp = ros::Time::now();
    _m_start.action = visualization_msgs::Marker::ADD;
    _m_start.ns = "map";
    _m_start.id = _id_start;
    _m_start.type = visualization_msgs::Marker::POINTS;
    _m_start.pose.orientation.w = 1.0;
    _m_start.scale.x = 1.0;
    _m_start.color.g = 1.0;
    _m_start.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = point[0];
    p.y = point[1];
    p.z = 0;
    _m_start.points.clear();
    _m_start.points.push_back(p);
}

void MapInfo::set_end(KDPoint &point)
{
    pt_end.assign(point.begin(), point.end());

    _m_end.header.frame_id = "/my_frame";
    _m_end.header.stamp = ros::Time::now();
    _m_end.action = visualization_msgs::Marker::ADD;
    _m_end.ns = "map";
    _m_end.id = _id_end;
    _m_end.type = visualization_msgs::Marker::POINTS;
    _m_end.pose.orientation.w = 1.0;
    _m_end.scale.x = 1.0;
    _m_end.color.r = 1.0;
    _m_end.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = point[0];
    p.y = point[1];
    p.z = 0;
    _m_end.points.clear();
    _m_end.points.push_back(p);
}

void MapInfo::set_path(std::vector<KDPoint> &path)
{
    _m_path.header.frame_id = "/my_frame";
    _m_path.header.stamp = ros::Time::now();
    _m_path.action = visualization_msgs::Marker::ADD;
    _m_path.ns = "map";
    _m_path.id = _id_path;
    _m_path.type = visualization_msgs::Marker::LINE_STRIP;
    _m_path.pose.orientation.w = 1.0;
    _m_path.scale.x = 0.2;
    _m_path.color.r = 1.0;
    _m_path.color.a = 1.0;

    _m_path.points.clear();
    for (auto p : path)
    {
        geometry_msgs::Point p_;
        p_.x = p[0];
        p_.y = p[1];
        p_.z = 0;
        _m_path.points.push_back(p_);
    }
    _marker_pub.publish(_m_path);
}

void MapInfo::set_openlist(std::vector<KDPoint> &points)
{
    _m_openlist.header.frame_id = "/my_frame";
    _m_openlist.header.stamp = ros::Time::now();
    _m_openlist.action = visualization_msgs::Marker::ADD;
    _m_openlist.ns = "map";
    _m_openlist.id = _id_openlist;
    _m_openlist.type = visualization_msgs::Marker::POINTS;
    _m_openlist.pose.orientation.w = 1.0;
    _m_openlist.scale.x = 0.3;
    _m_openlist.color.b = 0.5;
    _m_openlist.color.g = 0.5;
    _m_openlist.color.a = 1.0;

    _m_openlist.points.clear();
    for (auto p : points)
    {
        geometry_msgs::Point p_;
        p_.x = p[0];
        p_.y = p[1];
        p_.z = 0;
        _m_openlist.points.push_back(p_);
    }
    _marker_pub.publish(_m_openlist);
    _pub_i = (_pub_i + 1) % 10;
    if (_pub_i == 0)
        ros::Duration(0.01).sleep();
}

void MapInfo::set_closelist(std::vector<KDPoint> &points)
{
    _m_closelist.header.frame_id = "/my_frame";
    _m_closelist.header.stamp = ros::Time::now();
    _m_closelist.action = visualization_msgs::Marker::ADD;
    _m_closelist.ns = "map";
    _m_closelist.id = _id_closelist;
    _m_closelist.type = visualization_msgs::Marker::POINTS;
    _m_closelist.pose.orientation.w = 1.0;
    _m_closelist.scale.x = 0.3;
    _m_closelist.color.b = 1.0;
    _m_closelist.color.a = 1.0;

    _m_closelist.points.clear();
    for (auto p : points)
    {
        geometry_msgs::Point p_;
        p_.x = p[0];
        p_.y = p[1];
        p_.z = 0;
        _m_closelist.points.push_back(p_);
    }
    _marker_pub.publish(_m_closelist);
    _pub_i = (_pub_i + 1) % 10;
    if (_pub_i == 0)
        ros::Duration(0.01).sleep();
}

void MapInfo::set_rand_points(std::vector<KDPoint> &points)
{
    _m_rand_points.header.frame_id = "/my_frame";
    _m_rand_points.header.stamp = ros::Time::now();
    _m_rand_points.action = visualization_msgs::Marker::ADD;
    _m_rand_points.ns = "map";
    _m_rand_points.id = _id_rand_points;
    _m_rand_points.type = visualization_msgs::Marker::POINTS;
    _m_rand_points.pose.orientation.w = 1.0;
    _m_rand_points.scale.x = 0.3;
    _m_rand_points.color.b = 0.8;
    _m_rand_points.color.r = 0.8;
    _m_rand_points.color.a = 1.0;

    _m_rand_points.points.clear();
    for (auto p : points)
    {
        geometry_msgs::Point p_;
        p_.x = p[0];
        p_.y = p[1];
        p_.z = 0;
        _m_rand_points.points.push_back(p_);
    }
    _marker_pub.publish(_m_rand_points);
}

void MapInfo::set_roadmap(std::vector<std::pair<KDPoint, std::vector<KDPoint>>> &road_map)
{
    _m_roadmap.header.frame_id = "/my_frame";
    _m_roadmap.header.stamp = ros::Time::now();
    _m_roadmap.action = visualization_msgs::Marker::ADD;
    _m_roadmap.ns = "map";
    _m_roadmap.id = _id_roadmap;
    _m_roadmap.type = visualization_msgs::Marker::LINE_LIST;
    _m_roadmap.pose.orientation.w = 1.0;
    _m_roadmap.scale.x = 0.1;
    _m_roadmap.color.b = 0.3;
    _m_roadmap.color.r = 0.1;
    _m_roadmap.color.a = 1.0;

    _m_roadmap.points.clear();
    for (auto rm : road_map)
    {
        geometry_msgs::Point p1;
        p1.x = rm.first[0];
        p1.y = rm.first[1];
        p1.z = 0;
        for (auto p : rm.second)
        {
            geometry_msgs::Point p2;
            p2.x = p[0];
            p2.y = p[1];
            p2.z = 0;
            _m_roadmap.points.push_back(p1);
            _m_roadmap.points.push_back(p2);
        }
    }
    _marker_pub.publish(_m_roadmap);
}

bool MapInfo::Collision(KDPoint &point)
{
    if ((point[0] > 0) && (point[0] < _width) && (point[1] > 0) && (point[1] < _height))
    {
        std::vector<std::pair<KDPoint, double>> result;
        _okdtree.Query(point, 1, result);
        return (result.begin()->second < 1.0);
    }
    else
    {
        return true;
    }
}

bool MapInfo::Collision(KDPoint &p1, KDPoint &p2)
{
    if (Collision(p1) || Collision(p2))
        return true;
    std::vector<KDPoint> ps;
    ps.push_back(p1);
    ps.push_back(p2);
    double d = Distance(p1, p2);
    while (ps.size() < d * 1.3)
    {
        int i = 0;
        int j = 1;
        while (j < ps.size())
        {
            ps.insert(ps.begin() + j, MiddlePoint(ps[i], ps[j]));
            i += 2;
            j += 2;
        }
    }
    for (auto p : ps)
    {
        std::vector<std::pair<KDPoint, double>> result;
        _okdtree.Query(p, 1, result);
        if (result.begin()->second < 1.0)
            return true;
    }
    return false;
}

void MapInfo::ShowMap(void)
{
    while (_marker_pub.getNumSubscribers() < 1)
    {
        sleep(1);
    }
    
    _marker_pub.publish(_line_boundary);
    _marker_pub.publish(_obstacle);
    _marker_pub.publish(_m_start);
    _marker_pub.publish(_m_end);
}
