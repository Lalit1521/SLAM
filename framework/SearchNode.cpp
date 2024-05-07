#include "SearchNode.h"

using namespace std;

vector<vector2D> SearchNode::path()
{ //Returns from root to current node
    if (parent == nullptr)
    {
        vector<vector2D> path;
        path.push_back(state);
        return (path);
    }
    else
    {
        vector<vector2D> parent_path = parent->path();
        parent_path.push_back(state);
        return (parent_path);
    }
}

bool SearchNode::inPath(const vector2D &s)
{//If the current state is in the path from root to this node
    if (s == state)
        return (true);
    else
    {
        if (parent == nullptr)
            return (false);
        else
            return (parent->inPath(s));
    }
}

void SearchNode::printInfo()  // For Debugging
{
    if (parent == nullptr) 
    {
        printf("State:(x,y): (%d, %d) cost: %f m action: NULL parent: NULL\n ",  
            state.x, state.y, cost);
    }
    else
    {
        printf("State:(x,y): (%d, %d) cost: %f m action: (%d, %d) parent: (%d, %d)\n ",  
            state.x, state.y, cost, action.x, action.y, parent->state.x, parent->state.y);
    }
}

