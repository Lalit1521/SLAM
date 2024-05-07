#ifndef SCAN_ANALYSER_H_
#define SCAN_ANALYSER_H_

#include "LPoint2D.h"
#include "Scan2D.h"
#include <vector>

class ScanPointAnalyser
{
private:
    // Minimum distance to neighboring points [m]. If it is smaller than this, the error will be large, so do not use it for normal calculation.
    static const double FPDMIN;
    // Minimum distance to neighboring points [m]. If it is smaller than this, the error will be large, so do not use it for normal calculation.
    static const double FPDMAX;
    // Threshold for normal direction change [degrees]. If it is larger than this, it is considered a corner point.
    static const int CRTHRE = 45;
    static const int INVALID = -1;
    // Threshold for discrepancy between left and right normal directions
    double costh;

public:
    ScanPointAnalyser() : costh(cos(DEG2RAD(CRTHRE))) {}

    ~ScanPointAnalyser() {}

    //////////

    void analysePoints(std::vector<LPoint2D> &lps);
    bool calNormal(int idx, const std::vector<LPoint2D> &lps, int dir,
                   Vector2D &normal);
};

#endif