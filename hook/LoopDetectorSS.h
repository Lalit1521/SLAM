#ifndef LOOP_DETECTOR_SS_H_
#define LOOP_DETECTOR_SS_H_

#include "DataAssociator.h"
#include "LoopDetector.h"
#include "PointCloudMapLP.h"
#include "PoseEstimatorICP.h"
#include "PoseFuser.h"

class LoopDetectorSS : public LoopDetector
{
private:
    double radius;    //Search radius [m] (distance threshold between current position and revisit point)
    double atdthre;   //Threshold value of cumulative mileage difference [m]
    double scthre;    //ICP score threshold

    PointCloudMapLP *pcmap;  //Point Cloud map
    CostFunction *cfunc;     //Cost function (used separately from ICP)
    PoseEstimatorICP *estim; //Robot position estimator(ICP)
    DataAssociator *dass;    //data matcher
    PoseFuser *pfu;          //sensor fusion

public:
    LoopDetectorSS() : radius(4), atdthre(10), scthre(0.2) {}

    ~LoopDetectorSS() {}

    void setPoseEstimator(PoseEstimatorICP *p) { estim = p; }

    void setPoseFuser(PoseFuser *p) { pfu = p; }

    void setDataAssociator(DataAssociator *d) { dass = d; }

    void setCostFunction(CostFunction *f) { cfunc = f; }

    void setPointCloudMap(PointCloudMapLP *p) { pcmap = p; }

    virtual bool detectLoop(Scan2D *curScan, Pose2D &curPose, int cnt);
    void makeLoopArc(LoopInfo &info);
    bool estimateRevisitPose(const Scan2D *curScan, const std::vector<LPoint2D> &refLps,
                             const Pose2D &initPose, Pose2D &revisitPose);
};
#endif