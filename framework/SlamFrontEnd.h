
#ifndef SLAM_FRONT_END_H_
#define SLAM_FRONT_END_H_

#include "PointCloudMap.h"
#include "ScanMatcher2D.h"
#include "LoopDetector.h"
#include "PoseGraph.h"
#include "SlamBackEnd.h"
#include "PathPlanning.h"
#include "MotionPlanning.h"
#include <boost/circular_buffer.hpp>
#include <vector>

////////

// SLAM front end. Oversees robot position estimation, map generation, and loop
// closure.
class SlamFrontEnd
{
private:
    int cnt;          // logical time
    int keyframeSkip; // keyframe interval

    PointCloudMap *pcmap; // point cloud map
    ScanMatcher2D *smat;  // scan matching
    LoopDetector *lpd;    //Loop detector
    PoseGraph *pg;        //pose graph
    SlamBackEnd sback;    //SLAM backend
    PathPlanning path;
    MotionPlanning motion;

public:
    SlamFrontEnd() : cnt(0), keyframeSkip(10), smat(nullptr)
    {
        pg = new PoseGraph();
        sback.setPoseGraph(pg);
    }

    ~SlamFrontEnd() {}

    ///////
    void setLoopDetector(LoopDetector *l)
    {
        lpd = l;
        lpd->setPoseGraph(pg);
    }

    void setScanMatcher(ScanMatcher2D *s) { smat = s; }

    void setPointCloudMap(PointCloudMap *p) { pcmap = p; }

    void setRefScanMaker(RefScanMaker *r) { smat->setRefScanMaker(r); }

    //void setPathPlanner(PathPlanning *a) { path = a;}

    PointCloudMap *getPointCloudMap() { return (pcmap); }

    PoseGraph *getPoseGraph() { return (pg); }

    void setDgCheck(bool p) { smat->setDgCheck(p); }

    int getCnt() { return (cnt); }

    /////////

    void init();
    void process(Scan2D &scan);
    bool makeOdometryArc(Pose2D &curPose, const Eigen::Matrix3d &cov);

    void countLoopArcs();

};

#endif