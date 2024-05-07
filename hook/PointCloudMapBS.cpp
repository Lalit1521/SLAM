#include "PointCloudMapBS.h"

using namespace std;

//Add robot position
void PointCloudMapBS::addPose(const Pose2D &p)
{
    poses.emplace_back(p);
} 

//Add scan point cloud
void PointCloudMapBS::addPoints(const vector<LPoint2D> &lps)
{
    int skip = 5; //It's really heavy, so thinned it out to 1/5
    for (size_t i = 0; i < lps.size(); i += skip)
    {
        globalMap.emplace_back(lps[i]);  //Just add it to global Map
    }
}

// Global map generation. It's already done so don't do anything
void PointCloudMapBS::makeGlobalMap()
{
    printf("globalMap.size=%lu\n", globalMap.size()); // For confirmation
}

// Local map generation. dummy
void PointCloudMapBS::makeLocalMap()
{
    //  localMap = globalMap;
}

//dummy
void PointCloudMapBS::remakeMaps(const vector<Pose2D> &newPoses) {}

void PointCloudMapBS::makeOccupancyGridMap() {}