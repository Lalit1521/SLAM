#ifndef _POSE_OPTIMIZER_H_
#define _POSE_OPTIMIZER_H_

#include "CostFunction.h"
#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"
#include <vector>

class PoseOptimizer
{
    public:
        int allN;   //Total number of repeats for test
        double sum; //Total residual for test

    protected:
        double evthre; //Cost change threshold.If the amount of change is 
                       // less than this repitation ends
        double dd;     //Step of nuimerical differentiation (translation)
        double da;     //Numerical differentiation step (rotation)

        CostFunction *cfunc; //cost function

    public:
        PoseOptimizer(): evthre(0.000001), dd(0.00001), da(0.00001), cfunc(nullptr)
        {
            allN = 0;
            sum = 0;
        }

    ~PoseOptimizer() {}

    void setCostFunction(CostFunction *f) { cfunc = f; }

    void setEvlimit(double l) { cfunc->setEvlimit(l); }

    void setPoints(std::vector<const LPoint2D *> &curLps,
                   std::vector<const LPoint2D *> &refLps)
    {
        cfunc->setPoints(curLps, refLps);
    }

    void setEvthre(double inthre) { this->evthre = inthre; }
    double getPnrate() { return (cfunc->getPnrate());}

    void setDdDa(double d, double a)
    {
        dd = d;
        da = a;
    }

    virtual double optimizePose(Pose2D &initPose, Pose2D &estPose) = 0;
};

#endif