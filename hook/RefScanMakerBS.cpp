#include "RefScanMakerBS.h"

using namespace std;

const Scan2D *RefScanMakerBS::makeRefScan()
{
    vector<LPoint2D> &refLps = refScan.lps;  //Reference scan point cloud container
    refLps.clear();

    Pose2D lastPose = pcmap->getLastPose();  //Last estimated position saved in point cloud
    double(*R)[2] = lastPose.Rmat;
    double tx = lastPose.tx;
    double ty = lastPose.ty;

    //Make the last scan saved in the point cloud map the reference scan
    const vector<LPoint2D> &lps = pcmap->lastScan.lps;
    for (size_t i = 0; i < lps.size(); i++)
    {
        const LPoint2D &mp = lps[i];  //Reference scan points

        // Since the scan is in the robot coordinate system, convert it to the map coordinate system.
        LPoint2D rp;
        rp.x = R[0][0] * mp.x + R[0][1] * mp.y + tx;  // point position
        rp.y = R[1][0] * mp.x + R[1][1] * mp.y + ty;
        rp.nx = R[0][0] * mp.nx + R[0][1] * mp.ny;    //normal vector
        rp.ny = R[1][0] * mp.nx + R[1][1] * mp.ny;
        refLps.emplace_back(rp);
    }   
    return (&refScan); 
}