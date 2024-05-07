#include "ScanPointResampler.h"

using namespace std;

/////////

void ScanPointResampler::resamplePoints(Scan2D *scan)
{
    vector<LPoint2D> &lps = scan->lps; // scan point cloud
    
    if (lps.size() == 0)
        return;

    vector<LPoint2D> newLps; // Point cloud after resampling

    dis = 0; // dis is cumulative distance
    LPoint2D lp = lps[0];
    LPoint2D prevLp = lp;
    LPoint2D np(lp.sid, lp.x, lp.y);
    newLps.emplace_back(np); // dis is cumulative distance
    for (size_t i = 1; i < lps.size(); i++)
    {
        lp = lps[i]; // scan point
        bool inserted = false;

        bool exist = findInterpolatePoint(lp, prevLp, np, inserted);

        if (exist)
        {                            // There is a point to add
            newLps.emplace_back(np); // insert new point np
            prevLp = np;             // np becomes the previous point
            dis = 0;                 // Reset cumulative distance
            if (inserted)            // I put an interpolation point before lp, so I do lp again
                i--;
        }
        else
            prevLp = lp; // The current lp becomes the previous point
    }
    //printf("lps.size=%lu\n", lps.size());
    scan->setLps(newLps);

    printf("lps.size=%lu, newLps.size=%lu\n", lps.size(),
           newLps.size()); // For confirmation
}

bool ScanPointResampler::findInterpolatePoint(const LPoint2D &cp,
                                              const LPoint2D &pp, LPoint2D &np,
                                              bool &inserted)
{
    double dx = cp.x - pp.x;
    double dy = cp.y - pp.y;
    double L = sqrt(dx * dx + dy * dy); // For confirmation
    if (dis + L < dthreS)
    {             // Delete points whose predicted cumulative distance (dis+L) is smaller than dthreS
        dis += L; // add to dis
        return (false);
    }
    else if (
        dis + L >=
        dthreL)
    { // Points whose predicted cumulative distance is greater than dthreL are not interpolated and are left as is.
        np.setData(cp.sid, cp.x, cp.y);
    }
    else
    { // If the predicted cumulative distance exceeds dthreS, interpolate so that it becomes dthreS
        double ratio = (dthreS - dis) / L;
        double x2 = dx * ratio + pp.x; // Stretch out a little and the distance becomes dthreS
        double y2 = dy * ratio + pp.y;
        np.setData(cp.sid, x2, y2);
        inserted = true; // Flag indicating that np was placed before cp
    }

    return (true);
}