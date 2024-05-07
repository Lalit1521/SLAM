#ifndef MOTION_PLANNING_H_
#define MOTION_PLANNING_H_

#include "PathPlanning.h"
#include "PointCloudMap.h"

class MotionPlanning
{
public:
    static double KP; 
    static double LOOKAHEAD_DISTANCE; 
    double steering_angle;
    double speed;
    PointCloudMap *pcmap;

    void setPointCloudMap(PointCloudMap *p)
    {
        pcmap = p;
    }

    void command();

};
#endif