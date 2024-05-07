#ifndef LOOP_DETECTOR_H_
#define LOOP_DETECTOR_H_

#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"
#include "PoseGraph.h"
#include "Scan2D.h"
#include <vector>

struct LoopInfo
{
    bool arcked; //Does it has pose arc?
    int curId;   //Current keyframe id(scan)
    int refId;   //Reference keyframe id
    Pose2D pose; // Global pose where the current keyframe matches the reference keyframe (or vice versa if Grid-based)
    double score; //ICP matching score
    Eigen::Matrix3d cov; //covariance

    LoopInfo() : arcked(false), curId(-1), refId(-1), score(-1) {}

    ~LoopInfo() {}

    void setArcked(bool t) { arcked = t; }     
};

struct LoopMatch
{
    Scan2D curScan;
    Scan2D refScan;
    LoopInfo info;

    LoopMatch() {}

    LoopMatch(Scan2D &cs, Scan2D &rs, LoopInfo &i)
    {
        curScan = cs;
        refScan = rs;
        info = i;
    }
};

class LoopDetector
{
protected:
    PoseGraph *pg;       //pose graph
    std::vector<LoopMatch> loopMatches;

public:
    LoopDetector() {}

    ~LoopDetector() {}

    std::vector<LoopMatch> &getLoopMatches() { return (loopMatches); }

    void setPoseGraph(PoseGraph *p) { pg = p; }

    virtual bool detectLoop(Scan2D *curScan, Pose2D &curPose, int cnt);
};

#endif