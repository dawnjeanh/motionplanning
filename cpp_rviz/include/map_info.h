#ifndef __MAP_INFO__
#define __MAP_INFO__

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "kdtree.h"

class MapInfo
{
private:
    enum MarkerID
    {
        _id_boundary = 0,
        _id_obstacle,
        _id_start,
        _id_end,
        _id_path,
        _id_openlist,
        _id_closelist,
    };
    ros::Publisher _marker_pub;
    double _width, _height;
    visualization_msgs::Marker _line_boundary;
    visualization_msgs::Marker _obstacle;
    KDTree _okdtree;
    visualization_msgs::Marker _m_start;
    visualization_msgs::Marker _m_end;
    visualization_msgs::Marker _m_openlist;
    visualization_msgs::Marker _m_closelist;
    visualization_msgs::Marker _m_path;
    int _pub_i;
public:
    KDPoint pt_start;
    KDPoint pt_end;
    MapInfo(int argc, char** argv);
    ~MapInfo();

    void set_boundary(int w, int h);
    void set_obstacle(std::vector<KDPoint> &points);
    void set_start(KDPoint &point);
    void set_end(KDPoint &point);
    void set_path(std::vector<KDPoint> &path);
    void set_openlist(std::vector<KDPoint> &points);
    void set_closelist(std::vector<KDPoint> &points);
    bool Collision(KDPoint &point);
    void ShowMap(void);
};

#endif // !__MAP_INFO__
