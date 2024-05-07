#ifndef POINT_CLOUD_MAP_LP_H_
#define POINT_CLOUD_MAP_LP_H_

#include "PointCloudMap.h"
#include <boost/unordered_map.hpp>

// partial map
struct Submap
{
    double atdS;  // Cumulative mileage at the starting point of the partial map
    size_t cntS;  //First scan number of partial map
    size_t cntE;  //Last scan number of partial map

    std::vector<LPoint2D> mps;  //Scan point cloud in partial map

    Submap() : atdS(0), cntS(0), cntE(-1) {}

    Submap(double a, size_t s) : cntE(-1) 
    {
        atdS = a;
        cntS = s;
    }

    void addPoints(const std::vector<LPoint2D> &lps)
    {
        for (size_t i = 0; i < lps.size(); i++)
        {
            mps.emplace_back(lps[i]);
        }
    }

    std::vector<LPoint2D> subsamplePoints(int nthre);
};

// Point cloud map composed of partial maps
class PointCloudMapLP : public PointCloudMap
{
public:
    static double atdThre; //Cumulative driving distance(atd) [m] that separates partial maps
    double atd;            //Current accumulated travel distance
    std::vector<Submap> submaps; //partial map

public:
    PointCloudMapLP()
    {
        Submap submap;
        submaps.emplace_back(submap); //Create the first partial map
    }

    ~PointCloudMapLP() {}

    Pose2D getLastPose() const { return (lastPose); }

    void setLastPose(Pose2D &p) { lastPose = p; }

    std::vector<Submap> &getSubmaps() { return (submaps); }

    virtual void addPose(const Pose2D &p);
    virtual void addPoints(const std::vector<LPoint2D> &lps);
    virtual void makeGlobalMap();
    virtual void makeLocalMap();
    virtual void remakeMaps(const std::vector<Pose2D> &newPoses);
    virtual void makeOccupancyGridMap();
};

#endif

