#include "CostFunctionED.h"

using namespace std;

// ICP cost function by point distance
double CostFunctionED::calValue(double tx, double ty, double th)
{
    double a = DEG2RAD(th);
    double error = 0;
    int pn = 0;
    int nn = 0;
    for (size_t i = 0; i < curLps.size(); i++)
    {
        const LPoint2D *clp = curLps[i]; //Current scan point
        const LPoint2D *rlp = refLps[i]; //Reference scan point corresponding to clp

        double cx = clp->x;
        double cy = clp->y;

        double x = cos(a) * cx - sin(a) * cy + tx; //Convert to reference scan coordinate
        double y = sin(a) * cx + cos(a) * cy + ty;

        double edis = (x - rlp->x) * (x - rlp->x) + (y - rlp->y) * (y - rlp->y); // distance between points

        if (edis <= evlimit * evlimit)
            ++pn;   //Number of points with small error

        error += edis;   //Accumulate error at each point

        ++nn;
    }
    
    error = (nn > 0) ? error /nn : HUGE_VAL; // Take the average. If the valid score is 0, the value is HUGE_VAL
    pnrate = 1.0 * pn/nn;    //Ratio of points with small error
    error *= 100;
    return (error);
}