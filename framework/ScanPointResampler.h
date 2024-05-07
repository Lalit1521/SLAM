#ifndef SCAN_POINT_RESAMPLER_H_
#define SCAN_POINT_RESAMPLER_H_

#include "LPoint2D.h"
#include "MyUtil.h"
#include "Scan2D.h"
#include <vector>

class ScanPointResampler
{
private:
    double dthreS;    //Distance interval of points [m]
    double dthreL;    //Point distance threshold [m]. Do not interpolate if this interval exceeded
    double dis;

public:
    ScanPointResampler() : dthreS(0.05), dthreL(0.25), dis(0) {}

    ~ScanPointResampler() {}

    void setDthre(int s, int l)
    {
        dthreS = s;
        dthreL = l;
    }

    void resamplePoints(Scan2D *scan);
    bool findInterpolatePoint(const LPoint2D &cp, const LPoint2D &pp,
                              LPoint2D &np, bool &inserted);
};

#endif