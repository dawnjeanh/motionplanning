#include "kdtree.h"

void PrintKDPoint(KDPoint &p)
{
    std::cout << '(';
    for (auto it = p.begin(); it != p.end(); ++it)
    {
        std::cout << *it;
        if (it != p.end() - 1)
            std::cout << ", ";
    }
    std::cout << ')';
}

double distance(KDPoint &p1, KDPoint &p2)
{
    double d = 0.0;
    for (int i = 0; i < p1.size(); ++i)
        d += (p1[i] - p2[i]) * (p1[i] - p2[i]);
    return std::sqrt(d);
}

KDTree::KDTree(std::vector<KDPoint> &points)
{
    _points.assign(points.begin(), points.end());
    _dim = points[0].size();
    _proot = _CreatNode(0, points.size() - 1, 0, nullptr);
}

KDTree::~KDTree()
{
    _DeleteNode(_proot);
}

KDTree &KDTree::operator=(const KDTree &tree)
{
    if (_proot != nullptr)
    {
        _DeleteNode(_proot);
        _points.clear();
    }
    _points.assign(tree._points.begin(), tree._points.end());
    _dim = _points[0].size();
    _proot = _CreatNode(0, _points.size() - 1, 0, nullptr);
}

KDNode *KDTree::_CreatNode(int begin_idx, int end_idx, int dim, KDNode *p)
{
    if (begin_idx > end_idx)
        return nullptr;
    std::sort(_points.begin() + begin_idx, _points.begin() + end_idx + 1,
              [&](KDPoint &p1, KDPoint &p2){return p1[dim] < p2[dim];});
    int mid = (begin_idx + end_idx) / 2;
    KDNode *pnode = new KDNode();
    pnode->point.assign(_points[mid].begin(), _points[mid].end());
    pnode->dim_i = dim;
    pnode->parent = p;
    pnode->left = _CreatNode(begin_idx, mid - 1, (dim + 1) % _dim, pnode);
    pnode->right = _CreatNode(mid + 1, end_idx, (dim + 1) % _dim, pnode);
}

void KDTree::_DeleteNode(KDNode *p)
{
    if (p != nullptr)
    {
        _DeleteNode(p->left);
        _DeleteNode(p->right);
        delete p;
    }
}

void KDTree::_PrintNode(KDNode *p, int l)
{
    if (p == nullptr)
    {
        std::cout << "--> null" << std::endl;
    }
    else
    {
        if (l != 0)
            std::cout << "--> ";
        PrintKDPoint(p->point);
        std::cout << ' ' << l << ' ';
        _PrintNode(p->left, l + 1);
        _PrintNode(p->right, l + 1);
    }
}

void KDTree::Query(KDPoint &point, int k, std::vector<std::pair<KDPoint, double>> &result)
{
    int i = 0;
    KDNode *p = _proot;
    KDNode *pre = nullptr;
    std::vector<KDNode *> acess;
    while (true)
    {
        while (p->near(point) != nullptr)
        {
            p = p->near(point);
        }
        while (true)
        {
            auto it = std::find(acess.begin(), acess.end(), p);
            if (it != acess.end())
            {
                if (p != _proot)
                {
                    p = p->parent;
                    continue;
                }
                else
                {
                    return;
                }
            }
            acess.push_back(p);
            double dis = distance(point, p->point);
            if (result.size() < k)
                result.push_back(std::make_pair(p->point, dis));
            else
            {
                std::sort(result.begin(), result.end(),
                          [](std::pair<KDPoint, double> &pair1, std::pair<KDPoint, double> &pair2)
                          {
                              return pair1.second < pair2.second;
                          });
                if (dis < result.back().second)
                {
                    result.back().first.assign(p->point.begin(), p->point.end());
                    result.back().second = dis;
                }
            }
            if (p->havechild()) //p is not leaf
            {
                if (p->far(point) != nullptr)
                {
                    dis = std::abs(point[p->dim_i] - p->point[p->dim_i]);
                    std::sort(result.begin(), result.end(),
                              [](std::pair<KDPoint, double> &pair1, std::pair<KDPoint, double> &pair2)
                              {
                                  return pair1.second < pair2.second;
                              });
                    if (dis <= result.back().second)
                    {
                        p = p->far(point);
                        break;
                    }
                }
            }
            if (p != _proot)
            {
                p = p->parent;
            }
            else
            {
                return;
            }
        }
    }
}