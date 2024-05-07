#ifndef _POSE_OPTIMIZER_SL_H_
#define _POSE_OPTIMIZER_SL_H_

#include "PoseOptimizer.h"

class PoseOptimizerSL : public PoseOptimizer
{
public:
    PoseOptimizerSL() {}
    
    ~PoseOptimizerSL() {}

    virtual double optimizePose(Pose2D &initPose, Pose2D &estPose);
    double search(double ev0, Pose2D &pose, Pose2D &dp);
    double objFunc(double tt, Pose2D &pose, Pose2D &dp);
};

#endif