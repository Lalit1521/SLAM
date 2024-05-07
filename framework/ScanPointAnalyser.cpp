
#include "ScanPointAnalyser.h"

using namespace std;

const double ScanPointAnalyser::FPDMIN =
    0.06; // ScanPointResampler.shift with dthrS
const double ScanPointAnalyser::FPDMAX = 1.0;

///////////

// スFind the normal vector of the can point. Also, distinguish between straight lines, corners, and isolated cases.
void ScanPointAnalyser::analysePoints(vector<LPoint2D> &lps)
{
    for (int i = 0; i < lps.size(); i++)
    {
        LPoint2D &lp = lps[i]; // scan point
        ptype type;
        Vector2D nL, nR, normal;
        bool flagL =
            calNormal(i, lps, -1, nL); // nL is the normal vector obtained from lp and the point on the left side
        bool flagR =
            calNormal(i, lps, 1, nR); // nR is the normal vector obtained from lp and the point on the right side
        nR.x = -nR.x;                 // Match sign with nL
        nR.y = -nR.y;
        if (flagL)
        {
            if (flagR)
            { // Normal vectors can be calculated on both left and right sides
                if (fabs(nL.x * nR.x + nL.y * nR.y) >=
                    costh)
                {                // Normals on both sides are nearly parallel
                    type = LINE; // regarded as a straight line
                }
                else
                { // If it is far from parallel, it is considered a corner point.
                    type = CORNER;
                }
                // Average of normal vectors on both left and right sides
                double dx = nL.x + nR.x;
                double dy = nL.y + nR.y;
                double L = sqrt(dx * dx + dy * dy);
                normal.x = dx / L;
                normal.y = dy / L;
            }
            else
            { // The normal vector could only be taken on the left side
                type = LINE;
                normal = nL;
            }
        }
        else
        {
            if (flagR)
            { // I could only get the normal vector on the right side.
                type = LINE;
                normal = nR;
            }
            else
            {
                type = ISOLATE; // I couldn't get the normal vector on both sides. regarded as an isolated point
                normal.x = INVALID;
                normal.y = INVALID;
            }
        }

        lp.setNormal(normal.x, normal.y);
        lp.setType(type);

    
        //printf("Normal_Points x - %f , y - %f\n", normal.x, normal.y);
    }
}

// If the points on both sides of the point of interest cp are greater than or equal to dmin and less than or equal to dmax from cp, the normal is calculated.
bool ScanPointAnalyser::calNormal(int idx, const vector<LPoint2D> &lps, int dir,
                                  Vector2D &normal)
{
    const LPoint2D &cp = lps[idx]; // Highlights
    for (int i = idx + dir; i >= 0 && i < lps.size(); i += dir)
    {
        const LPoint2D &lp = lps[i]; // Point on dir (left or right) side of cp
        double dx = lp.x - cp.x;
        double dy = lp.y - cp.y;
        double d = sqrt(dx * dx + dy * dy);
        if (d >= FPDMIN && d <= FPDMAX)
        { // If the distance d between cp and lp is appropriate, calculate the normal
            normal.x = dy / d;
            normal.y = -dx / d;
            return (true);
        }

        if (d > FPDMAX) // I'll stop mid-way because I'm getting further away.
            break;
    }

    return (false);
}