#ifndef SCAN2D_H_
#define SCAN2D_H_

#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"
#include <vector>

struct Scan2D
{
    static double MAX_SCAN_RANGE; //Scan point distance value upper limit [m]
    static double MIN_SCAN_RANGE; //Scan point distance value lower limit [m]
    
    int sid;    //scan id
    Pose2D pose;  //Odometric values at time of scan acquisition
    std::vector<LPoint2D> lps; //scan point cloud

    Scan2D() : sid(0) {}
    ~Scan2D() {}

    void setSid(int s) {sid = s;}

    void setLps(const std::vector<LPoint2D> &ps) {lps = ps;}

    void setPose(Pose2D &p) { pose = p;}

};

#endif