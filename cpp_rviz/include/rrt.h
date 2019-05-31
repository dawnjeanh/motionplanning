#ifndef __RRT__
#define __RRT__

#include <vector>
#include <tuple>
#include <utility>
#include <cmath>
#include <functional>
#include <algorithm>
#include "kdtree.h"

class RRT
{
private:
    KDPoint _root;
    std::vector<std::pair<KDPoint, int>> _rrt;
public:
    class iterator
    {
    private:
        const std::vector<std::pair<KDPoint, int>>::iterator _it_begin;
        int _pos;
    public:
        iterator(const std::vector<std::pair<KDPoint, int>>::iterator begin, int pos) : _it_begin(begin), _pos(pos){}
        inline bool operator!=(iterator &it)
        {
            return (_pos != it._pos);
        }
        inline KDPoint operator*()
        {
            return (_it_begin + _pos)->first;
        }
        inline iterator& operator++()
        {
            ++_pos;
            return *this;
        }
    };
    RRT(){}
    void set_root(KDPoint &p);
    KDPoint SearchNearestVertex(KDPoint &q_rand);
    KDPoint CalcNewPoint(KDPoint &q_near, KDPoint &q_rand);
    void Add(KDPoint &q_new, KDPoint &q_near);
    KDPoint GetParent(KDPoint &p);
    double Cost(KDPoint &p);
    void Rewire(KDPoint &p, double r, std::function<bool (KDPoint &p1, KDPoint &p2)> Collision);
    inline int size(void)
    {
        return _rrt.size();
    }
    inline iterator begin()
    {
        return iterator(_rrt.begin(), 0);
    }
    inline iterator end()
    {
        return iterator(_rrt.begin(), size());
    }
};

#endif // !__RRT__