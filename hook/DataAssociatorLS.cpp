#include "DataAssociatorLS.h"
#include  <boost/timer.hpp>

using namespace std;

// Find the point corresponding to each scan point of the current scan curScan from baseLps
double DataAssociatorLS::findCorrespondence(const Scan2D *curScan,
                                            const Pose2D &predPose)
{
    boost::timer tim;  //For processing time measurement

    double dthre = 0.2; // Exclude points futher away this [m]
    curLps.clear();    // Empty the mapping current point cloud
    refLps.clear();    // Empty the mapping reference scan point cloud
    for (size_t i = 0; i < curScan->lps.size(); i++)
    {
        const LPoint2D *clp = &(curScan->lps[i]); //Current scan point with a pointer
        // Find the point closest to the coordinate transformation of scan point lp using predPose
        LPoint2D glp;                //predicted position of crp
        predPose.globalPoint(*clp, glp);  //Coordinate transformation with predPose

        double dmin = HUGE_VAL;       //Minimum distance
        const LPoint2D *rlpmin = nullptr;  //Closest point
        for (size_t j = 0; j < baseLps.size(); j++)
        {
            const LPoint2D *rlp = baseLps[j];  // Reference scan point
            double d = (glp.x - rlp->x) * (glp.x - rlp->x) +
                        (glp.y - rlp->y) * (glp.y - rlp->y);
            if (d <= dthre * dthre && d < dmin)
            {
                //Save the point with minimum distance within dthre
                dmin = d;
                rlpmin = rlp;
            }
        }

        if (rlpmin != nullptr)
        {
            //Register if there is a nearest point
            curLps.push_back(clp);
            refLps.push_back(rlpmin);
        } 
    }
    //for (size_t i = 0; i < curLps.size(); i++)
    //{
    //    printf("cur_corr Points x - %f , y - %f\n", curLps[i]->x, curLps[i]->y);
    //}
    double ratio = (1.0 * curLps.size()) / curScan -> lps.size(); //Ratio of matched points

    return (ratio);
}