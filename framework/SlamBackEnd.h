#ifndef SLAM_BACK_END_H_
#define SLAM_BACK_END_H_

#include "PointCloudMap.h"
#include "PoseGraph.h"
#include <vector>

class SlamBackEnd
{
private:
    std::vector<Pose2D> newPoses;  //Posture after Pose Adjustment
    PointCloudMap *pcmap;          //Point cloud map
    PoseGraph *pg;                 //Pose graph

public:
    SlamBackEnd() {}

    ~SlamBackEnd() {}

    void setPointCloudMap(PointCloudMap *m) { pcmap = m; }

    void setPoseGraph(PoseGraph *g) { pg = g; }

    Pose2D adjustPoses();
    void remakeMaps();
};

#endif