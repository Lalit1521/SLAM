#ifndef PATH_PLANNING_H_
#define PATH_PLANNING_H_

#include "PointCloudMap.h"
#include "PriorityQueue.h"
#include "SearchNode.h"
#include "MyUtil.h"
#include <vector>
#include <map>

class PathPlanning
{
private:
    PointCloudMap *pcmap;
    std::vector<vector2D> legalInputs;
    std::pair<vector2D, double> successor(vector2D &state, vector2D &action);
    bool legal(const vector2D &state, const vector2D &action);
    double heruisticCalc(vector2D &newS, vector2D &goalPoint);
    bool occupied(std::pair<int, int> position);
    bool robotCanOccupy(std::pair<int, int> position);

public:
    PriorityQueue agenda;

public:
    PathPlanning()
    {
        for (int dx: {-1, 0, 1}){
            for (int dy : {-1, 0, 1}){
                if (dx != 0 || dy != 0){
                    legalInputs.push_back(vector2D(dx, dy));
                }
            }
        }
    }

    ~PathPlanning() {};

    std::vector<vector2D> search(vector2D &initalState, vector2D &goalTest, int maxNodes = 100000);
    
    void setPointCloudMap(PointCloudMap *p)
    {
        pcmap = p;
    }

    void planning();
    void loopclosurePlan();
    void planToGoal(Pose2D &goalPoint);


};
#endif