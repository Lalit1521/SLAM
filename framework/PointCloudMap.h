#ifndef POINT_CLOUD_MAP_H_
#define POINT_CLOUD_MAP_H_

#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "NNGridTable.h"
#include <vector>

// Base class for point cloud maps
class PointCloudMap
{
    public:
        static const int MAX_POINT_NUM = 1000000; //Maximum number of globalMap points
    
    int nthre;      //Lattice table cell point threshold (GT and LP only)

    std::vector<Pose2D> poses;  //robot trajectory
    Pose2D lastPose;            //Last estimated robot position
    Scan2D lastScan;            //Last processed scan

    std::vector<LPoint2D> globalMap; //Overall map. Points after thinning
    std::vector<LPoint2D> localMap;  // Local map near your current location. Use for scan matching
    NNGridTable nntabLocal;
    std::vector<Vector2D> plan;

    PointCloudMap():nthre(1)
    {
        globalMap.reserve(MAX_POINT_NUM);
    }

    ~PointCloudMap() {}

    void setNthre(int n) { nthre = n; }

    void setLastPose(const Pose2D &p) {lastPose = p;}

    Pose2D getLastPose() const { return (lastPose); }

    void setLastScan(const Scan2D &s) {lastScan = s;}

    virtual void addPose(const Pose2D &p) = 0;
    virtual void addPoints(const std::vector<LPoint2D> &lps) = 0;
    virtual void makeGlobalMap() = 0;
    virtual void makeLocalMap() = 0;
    virtual void remakeMaps(const std::vector<Pose2D> &newPoses) = 0;
    virtual void makeOccupancyGridMap() = 0;
};

#endif