#ifndef POINT_CLOUD_MAP_GT_H_
#define POINT_CLOUD_MAP_GT_H_

#include "NNGridTable.h"
#include "PointCloudMap.h"
#include <boost/unordered_map.hpp>

class PointCloudMapGT : public PointCloudMap
{
public:
    std::vector<LPoint2D> allLps;    // full scan point cloud
    NNGridTable nntab;               // lattice table

public:
    PointCloudMapGT()
    {
        allLps.reserve(MAX_POINT_NUM);
    }

    ~PointCloudMapGT() {}

    virtual void addPose(const Pose2D &p);
    virtual void addPoints(const std::vector<LPoint2D> &lps);
    virtual void makeGlobalMap();
    virtual void makeLocalMap();
    void subsamplePoints(std::vector<LPoint2D> &sps);
    virtual void remakeMaps(const std::vector<Pose2D> &newPoses);
    virtual void makeOccupancyGridMap();

};

#endif