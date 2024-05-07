#ifndef SEARCH_NODE_H_
#define SEARCH_NODE_H_

#include "LPoint2D.h"
#include <vector>


class SearchNode
{
public:
    vector2D action;
    vector2D state;
    double cost;
    SearchNode *parent;

public:
    SearchNode(const vector2D &st, const vector2D &act, SearchNode *p, double c) 
        : state(st), action(act), parent(p)
    {
        if (parent == nullptr)
            cost = c;
        else
            cost = parent->cost + c;
    }

    ~SearchNode() {}

    std::vector<vector2D>  path();
    bool inPath(const vector2D &s);

    void printInfo();  // For Debugging
};
#endif