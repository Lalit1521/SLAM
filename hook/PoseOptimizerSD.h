#ifndef _POSE_OPTIMIZER_SD_H_
#define _POSE_OPTIMIZER_SD_H_

#include "PoseOptimizer.h"

//Minimize the cost function using steepest descent
class PoseOptimizerSD : public PoseOptimizer
{
    public:
        PoseOptimizerSD() {}

        ~PoseOptimizerSD() {}

        virtual double optimizePose(Pose2D &initPose, Pose2D &estPose);        
};

#endif
