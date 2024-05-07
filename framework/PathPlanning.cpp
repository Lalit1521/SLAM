#include "PathPlanning.h"
#include <algorithm>

using namespace std;

vector<vector2D> PathPlanning::search(vector2D &initalState, vector2D &goalTest, int maxNodes)
{
    vector2D action;
    SearchNode startNode(initalState, action, nullptr, 0.0); //Intial Node
    if (goalTest == initalState)   //If Inital Node is the goal return the path
    {
        return startNode.path();
    }
    else
    {
        auto p = make_pair(&startNode, 0.0);
        agenda.push(p);  //Push the Node to the agenda
        map<vector2D, bool> expanded; //Initialize the dictionary for DP
        int count = 1;                               
        while (!agenda.isEmpty() && maxNodes > count)  
        {   
            SearchNode *n = agenda.pop();  //Pop the least cost Node from the agenda 
            //n->printInfo();
            if (expanded.find(n->state) == expanded.end()) //Check whether we have visited(popped) this state already
            {
                expanded[n->state] = true; 
                if (goalTest == n->state)  
                { //If goal Node is reached return the path
                    printf("count = %d\n", count);
                    return (n->path());
                }
                for (auto& a : legalInputs)
                {
                    pair<vector2D, double> newS = successor(n->state, a); //Get the valid next state for the action(leaglInputs). 
                    if (expanded.find(newS.first) == expanded.end()) //Check whether we have visited(popped) this state already
                    {
                        count += 1;
                        double heuristic = heruisticCalc(newS.first, goalTest); //Calculate Heuristic
                        SearchNode *newN = new SearchNode(newS.first, a, n, newS.second);  //Dynamically create newN since it gets out of scope.
                        //newN->printInfo();
                        pair<SearchNode*, double> newNode = make_pair(newN, newN->cost + heuristic);
                        agenda.push(newNode); //Push the node to the agenda.
                    }
                }
            }
        }
        printf("Search failed after visiting %d states\n", count);
        return {};
    }
}

pair<vector2D, double> PathPlanning::successor(vector2D &state, vector2D &action)
{//Returns the next state for the given action.
    double csize = pcmap->nntabLocal.csize;
    vector2D newState(state.x + action.x, state.y + action.y); 
    double delta = sqrt(pow((action.x * csize),2) + pow((action.y * csize),2)); //Calculates cost for the action
    if (!legal(state, newState))    //Check whether the action is legal or not
        return make_pair(state, delta); //If not legal then return same state and action cost for that move
    else
        return make_pair(newState, delta);//Return newState and action cost for that move
}

bool PathPlanning::legal(const vector2D &state, const vector2D &action)
{
    //When your current and target squares are free, but one of the other two squares that are adjacent to both the current
    //and target squares is occupied, it is possible that the robot will have a collision. Such a move
    //should be treated in the same way as attempting to move into a square that is occupied.
    int tsize = pcmap->nntabLocal.width;
    if (state.x < 0 || state.y < 0 || state.x > tsize || state.y > tsize)
        return false;
    int minX = min(state.x, action.x);
    int maxX = max(state.x, action.x);
    int minY = min(state.y, action.y);
    int maxY = max(state.y, action.y);
    
    for (int x = minX; x <= maxX; x++)
    {
        for (int y = minY; y <= maxY; y++)
        {
            if ((x != state.x || y != state.y) && !robotCanOccupy(make_pair(x, y)))
                return false;
        }
    }
    return true;
}

bool PathPlanning::occupied(pair<int, int> position) 
{
    //Check whether the current grid square is occupied by obstacles.
    //Return True if it is occupied by a obstacles.
    int xIndex = position.first;
    int yIndex = position.second;
    int tsize = pcmap->nntabLocal.width;
    size_t index = (yIndex * tsize) + xIndex;
    return (xIndex < 0 || yIndex < 0 || xIndex >= 1600 || yIndex >= 1600 || pcmap->nntabLocal.occupied(index));
}

double PathPlanning::heruisticCalc(vector2D &newS, vector2D &goalIndices)
{//Calcuate the Heuristic cost.This is based on Euclidean distance.
    Vector2D Goalpos;
    pcmap->nntabLocal.indicesToPoint(goalIndices, Goalpos);
    Vector2D curPose;
    pcmap->nntabLocal.indicesToPoint(newS, curPose);
    double cost = sqrt((Goalpos.x - curPose.x) * (Goalpos.x - curPose.x) + (Goalpos.y - curPose.y) * (Goalpos.y - curPose.y));
    return (cost);
}

bool PathPlanning::robotCanOccupy(pair<int, int> position)
{//Check whether robot can occupy
    int xIndex = position.first;
    int yIndex = position.second;
    for (int dx = 0; dx <= 4; ++dx)
    {
        for (int dy = 0; dy <= 4; ++dy)
        {
            int xPlus = MyUtil::clamp(xIndex + dx, 0, 1600 - 1);
            int xMinus = MyUtil::clamp(xIndex - dx, 0, 1600 - 1);
            int yPlus = MyUtil::clamp(yIndex + dy, 0, 1600 - 1);
            int yMinus = MyUtil::clamp(yIndex - dy, 0, 1600 -1);
                
            if (occupied(make_pair(xPlus, yPlus)) || occupied(make_pair(xPlus, yMinus)) ||
                occupied(make_pair(xMinus, yPlus)) || occupied(make_pair(xMinus, yMinus)))
                {
                    return false;
                }
        }
    }
    return true;
}

void PathPlanning::planning()
{
    size_t nodes = pcmap->poses.size();
    Pose2D curNode = pcmap->poses[nodes-2];
    Pose2D goalNode = pcmap->poses.back();
    vector2D curPose;
    pcmap->nntabLocal.pointToIndices(curNode, curPose);
    vector2D goalPose;
    pcmap->nntabLocal.pointToIndices(goalNode, goalPose);

    agenda.clear();
        
    vector<vector2D> paths = search(curPose, goalPose);
    for (int i = 0; i < paths.size(); i++)
    {   
        Vector2D pos;
        pcmap->nntabLocal.indicesToPoint(paths[i], pos);
        pcmap->plan.push_back(pos);
    }
}

void PathPlanning::loopclosurePlan()
{   
    agenda.clear();
    pcmap->plan.clear();
    for (int i = 1; i < pcmap->poses.size(); i++)
    {
        Pose2D curNode = pcmap->poses[i-1];
        Pose2D goalNode = pcmap->poses[i];
        vector2D curPose;
        pcmap->nntabLocal.pointToIndices(curNode, curPose);
        vector2D goalPose;
        pcmap->nntabLocal.pointToIndices(goalNode, goalPose);

        vector<vector2D> paths = search(curPose, goalPose);
        for (int i = 0; i < paths.size(); i++)
        {   
            Vector2D pos;
            pcmap->nntabLocal.indicesToPoint(paths[i], pos);
            pcmap->plan.push_back(pos);
        }
    }
}

void PathPlanning::planToGoal(Pose2D &goalPoint)
{
    Pose2D curNode = pcmap->poses.back();
    Pose2D goalNode = goalPoint;
    vector2D curPose;
    pcmap->nntabLocal.pointToIndices(curNode, curPose);
    vector2D goalPose;
    pcmap->nntabLocal.pointToIndices(goalNode, goalPose);

    agenda.clear();
    pcmap->plan.clear();
        
    vector<vector2D> paths = search(curPose, goalPose);
    for (int i = 0; i < paths.size(); i++)
    {   
        Vector2D pos;
        pcmap->nntabLocal.indicesToPoint(paths[i], pos);
        pcmap->plan.push_back(pos);
    }
}