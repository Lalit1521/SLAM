#include "DataAssociatorGT.h"
#include <boost/timer.hpp>

using namespace std;

double DataAssociatorGT::findCorrespondence(const Scan2D *curScan, const Pose2D &predPose)
{
    boost::timer tim;   //For proccessing time measurement

    curLps.clear();    //Empty the mapping cuirrent scan point cloud
    refLps.clear();    //Empty the mapping reference scan point cloud

    for (size_t i = 0; i < curScan->lps.size(); i++)
    {
        const LPoint2D *clp = &(curScan->lps[i]);

        //Find the nearest Point using a lattice table.
        const LPoint2D *rlp = nntab.findClosestPoint(clp, predPose);

        if (rlp != nullptr)
        {
            curLps.push_back(clp);
            refLps.push_back(rlp);
        } 
    }

    double ratio = (1.0 * curLps.size()) / curScan->lps.size(); //Ratio of matched points

    return (ratio);
}