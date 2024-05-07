#include "PointCloudMapLP.h"
#include "NNGridTable.h"

using namespace std;

double PointCloudMapLP::atdThre = 10;

///////////

// Obtain representative points of partial map using grid table
vector<LPoint2D> Submap::subsamplePoints(int nthre)
{
    NNGridTable nntab; // lattice table
    for (size_t i = 0; i < mps.size(); i++)
    {
        LPoint2D &lp = mps[i];
        nntab.addPoint(&lp); // Register all points
    }

    vector<LPoint2D> sps;
    nntab.makeCellPoints(nthre, sps); // Put representative points of nthre or more cells into sps
    printf("mps.size=%lu, sps.size=%lu\n", mps.size(), sps.size());

    return (sps);
}

/////////

// Add robot position
void PointCloudMapLP::addPose(const Pose2D &p)
{
    // Calculation of cumulative mileage (ATD)
    if (poses.size() > 0)
    {
        Pose2D pp = poses.back();
        atd +=
            sqrt((p.tx - pp.tx) * (p.tx - pp.tx) + (p.ty - pp.ty) * (p.ty - pp.ty));
    }
    else
    {
        atd += sqrt(p.tx * p.tx + p.ty * p.ty);
    }

    poses.emplace_back(p);
}

// Add scan points
void PointCloudMapLP::addPoints(const vector<LPoint2D> &lps)
{
    Submap &curSubmap = submaps.back(); // Current partial map
    if (atd - curSubmap.atdS >=
        atdThre)
    { // When the cumulative mileage exceeds the threshold, change to a new partial map
        size_t size = poses.size();
        curSubmap.cntE = size - 2; // Last scan number of partial map
        curSubmap.mps = curSubmap.subsamplePoints(
            nthre); // Only the representative points of the completed partial map are used (reduced weight)
        size = size - 1;
        Submap submap(atd, size);     // new partial map
        submap.addPoints(lps);        // Registration of scan point cloud
        submaps.emplace_back(submap); // Add partial map
    }
    else
    {
        curSubmap.addPoints(lps); // Add point cloud to current submap
    }
}

// Generation of global map. It would be faster to create the local map here as well.
void PointCloudMapLP::makeGlobalMap()
{
    globalMap.clear(); // Initialization
    localMap.clear();
    // Collect points from already determined submaps other than the current one
    for (size_t i = 0; i < submaps.size() - 1; i++)
    {
        Submap &submap = submaps[i]; // partial map
        vector<LPoint2D> &mps =
            submap.mps; // Point cloud of partial map. There are only representative points
        for (size_t j = 0; j < mps.size(); j++)
        {
            globalMap.emplace_back(mps[j]); // Insert all points on the overall map
        }
        if (i == submaps.size() - 2)
        { // Only the last partial map is included in the local map.
            for (size_t j = 0; j < mps.size(); j++)
            {
                localMap.emplace_back(mps[j]);
            }
        }
    }

    // Insert the representative point of the current partial map into the global map and local map
    Submap &curSubmap = submaps.back();                      // Current partial map
    vector<LPoint2D> sps = curSubmap.subsamplePoints(nthre); // get representative points
    for (size_t i = 0; i < sps.size(); i++)
    {
        globalMap.emplace_back(sps[i]);
        localMap.emplace_back(sps[i]);
    }

    // The following is for confirmation
    printf("curSubmap.atd=%g, atd=%g, sps.size=%lu\n", curSubmap.atdS, atd,
           sps.size());
    printf("submaps.size=%lu, globalMap.size=%lu\n", submaps.size(),
           globalMap.size());
}

// Generating local maps
void PointCloudMapLP::makeLocalMap()
{
    localMap.clear(); // Initialization
    if (submaps.size() >= 2)
    {
        Submap &submap = submaps[submaps.size() - 2]; // Use only the previous partial map
        vector<LPoint2D> &mps =
            submap.mps; // Point cloud of partial map. There are only representative points
        for (size_t i = 0; i < mps.size(); i++)
        {
            localMap.emplace_back(mps[i]);
        }
    }

    // Put the representative point of the current partial map into the local map
    Submap &curSubmap = submaps.back();                      // Current partial map
    vector<LPoint2D> sps = curSubmap.subsamplePoints(nthre); // get representative points
    for (size_t i = 0; i < sps.size(); i++)
    {
        localMap.emplace_back(sps[i]);
    }

    printf("localMap.size=%lu\n", localMap.size()); // For confirmation
}

//////////

// Rebuild the map using the robot trajectory newPose after pose adjustment
void PointCloudMapLP::remakeMaps(const vector<Pose2D> &newPoses)
{
    // Modify the position of points within each submap
    for (size_t i = 0; i < submaps.size(); i++)
    {
        Submap &submap = submaps[i];
        vector<LPoint2D> &mps =
            submap.mps; // Point cloud of partial map. Currently, areas other than the map are representative points.
        for (size_t j = 0; j < mps.size(); j++)
        {
            LPoint2D &mp = mps[j];
            size_t idx = mp.sid; // point scan number
            if (idx >= poses.size())
            { // Invalid scan number (bug if there is one)
                continue;
            }

            const Pose2D &oldPose = poses[idx];    // Old robot position corresponding to mp
            const Pose2D &newPose = newPoses[idx]; // New robot position corresponding to mp
            const double(*R1)[2] = oldPose.Rmat;
            const double(*R2)[2] = newPose.Rmat;
            LPoint2D lp1 =
                oldPose.relativePoint(mp); // Convert mp to sensor coordinate system with oldPose
            LPoint2D lp2 =
                newPose.globalPoint(lp1); // Convert to map coordinate system after pose adjustment with newPose
            mp.x = lp2.x;
            mp.y = lp2.y;
            double nx = R1[0][0] * mp.nx +
                        R1[1][0] * mp.ny; // The normal vector is also converted to the sensor coordinate system using oldPose.
            double ny = R1[0][1] * mp.nx + R1[1][1] * mp.ny;
            double nx2 =
                R2[0][0] * nx +
                R2[0][1] *
                    ny; // The normal vector is also converted to the map coordinate system after pose adjustment using newPose.
            double ny2 = R2[1][0] * nx + R2[1][1] * ny;
            mp.setNormal(nx2, ny2);
        }
    }

    makeGlobalMap(); // Generate global and local maps from partial maps

    for (size_t i = 0; i < poses.size(); i++)
    { // Update poses to the value after pose adjustment
        poses[i] = newPoses[i];
    }
    lastPose = newPoses.back();
}

void PointCloudMapLP::makeOccupancyGridMap()
{
    nntabLocal.clear(); // lattice table
    for (size_t i = 0; i < globalMap.size(); i++)
    {
        LPoint2D &lp = globalMap[i];
        nntabLocal.addPoint(&lp); // Register all points
    }

}