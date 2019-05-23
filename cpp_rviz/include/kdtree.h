#ifndef __KDTREE__
#define __KDTREE__

#include <iostream>
#include <vector>
#include <algorithm>
#include <tuple>
#include <utility>
#include <stack>
#include <cmath>

typedef std::vector<double> KDPoint;
#define E (0.00001)
void PrintKDPoint(KDPoint &p);
double Distance(KDPoint &p1, KDPoint &p2);
bool Equal(double a, double b);
bool Equal(KDPoint &p1, KDPoint &p2);
bool operator==(KDPoint &p1, KDPoint &p2);
bool operator!=(KDPoint &p1, KDPoint &p2);
KDPoint MiddlePoint(KDPoint &p1, KDPoint &p2);

struct KDNode
{
    KDPoint point;
    KDNode *left;
    KDNode *right;
    KDNode *parent;
    int dim_i;
    KDNode *near(KDPoint &p)
    {
        return (p[dim_i] <= point[dim_i]) ? left : right;
    }
    KDNode *far(KDPoint &p)
    {
        return (p[dim_i] <= point[dim_i]) ? right : left;
    }
    bool havechild(void)
    {
        return ((left != nullptr) || (right != nullptr));
    }
};

class KDTree
{
private:
    int _dim;
    std::vector<KDPoint> _points;
    KDNode *_proot;
    KDNode *_CreatNode(int begin_idx, int end_idx, int dim, KDNode *p);
    void _DeleteNode(KDNode *p);
    void _PrintNode(KDNode *p, int l);
public:
    KDTree(void):_proot(nullptr){}
    KDTree(std::vector<KDPoint> &points);
    ~KDTree();
    KDTree &operator=(const KDTree &tree);
    void Query(KDPoint &point, int k, std::vector<std::pair<KDPoint, double>> &result);
    void PrintKDTree(void)
    {
        _PrintNode(_proot, 0);
    }
};

#endif // !__KDTREE__