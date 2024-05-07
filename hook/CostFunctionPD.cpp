#include "CostFunctionPD.h"

using namespace std;

// Cost function with vertical distance
double CostFunctionPD::calValue(double tx, double ty, double th)
{
    double a = DEG2RAD(th);

    double error = 0;
    int pn = 0;
    int nn = 0;
    for (size_t i = 0; i < curLps.size(); i++)
    {
        const LPoint2D *clp = curLps[i]; // Current scan point
        const LPoint2D *rlp = refLps[i]; // Reference scan point corresponding to clp

        if (rlp->type != LINE) // Do not use unless the point is on a straight line
            continue;

        double cx = clp->x;
        double cy = clp->y;
        double x =
            cos(a) * cx - sin(a) * cy + tx; // Convert clp to reference scan coordinate system
        double y = sin(a) * cx + cos(a) * cy + ty;

        double pdis = (x - rlp->x) * rlp->nx + (y - rlp->y) * rlp->ny; // vertical distance

        double er = pdis * pdis;
        if (er <= evlimit * evlimit)
            ++pn; // Number of points with small error

        error += er; // Accumulate error at each point
        ++nn;
    }

    error = (nn > 0) ? error / nn : HUGE_VAL; // If the valid score is 0, the value is HUGE_VAL
    pnrate = 1.0 * pn / nn;                   // Ratio of points with small error

    //  printf("CostFunctionPD: error=%g, pnrate=%g, evlimit=%g\n", error, pnrate,
    //  evlimit);     // For confirmation

    error *= 100; // Multiply by 100 to prevent the evaluation value from becoming too small.

    return (error);
}