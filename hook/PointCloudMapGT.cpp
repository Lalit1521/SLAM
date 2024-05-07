#include "PointCloudMapGT.h"

using namespace std;

//Add robot position
void PointCloudMapGT::addPose(const Pose2D &p)
{
    poses.emplace_back(p);
}

// Find the representative point of each cell in the grid table and store it in global map
void PointCloudMapGT::subsamplePoints(vector<LPoint2D> &sps)
{
    nntab.clear();     //Initializing the grid map
    for (size_t i = 0; i < allLps.size(); i++)
    {
        nntab.addPoint(&(allLps[i]));   //Register all points in grid map
    }

    nntab.makeCellPoints(nthre, sps);  //Get representative points from cell
    
    printf("allLps.size=%lu, sps.size=%lu\n", allLps.size(), sps.size());
}

// Add scan point cloud 
void PointCloudMapGT::addPoints(const vector<LPoint2D> &lps)
{
    for (size_t i = 0; i < lps.size(); i++)
    {
        allLps.emplace_back(lps[i]);
    }
}

//Generation of global map
void PointCloudMapGT::makeGlobalMap()
{
    globalMap.clear();
    subsamplePoints(globalMap); 

    printf("GT: globalMap.size=%lu\n", globalMap.size());
}

// Generating local maps. Use the overall map as is
void PointCloudMapGT::makeLocalMap()
{
    localMap = globalMap;
}

//dummy
void PointCloudMapGT::remakeMaps(const vector<Pose2D> &newPoses) {}

void PointCloudMapGT::makeOccupancyGridMap() {}