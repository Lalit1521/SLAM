#ifndef SCAN_MATCHER2D_H_
#define SCAN_MATCHER2D_H_

#include "LPoint2D.h"
#include "MyUtil.h"
#include "PointCloudMap.h"
#include "Pose2D.h"
#include "PoseEstimatorICP.h"
#include "PoseFuser.h"
#include "RefScanMaker.h"
#include "Scan2D.h"
#include "ScanPointResampler.h"
#include "ScanPointAnalyser.h"

#include <vector>

// Perform scan matching using ICP
class ScanMatcher2D
{
    private:
        int cnt;          // Logical time. Corresponds to scan number
        Scan2D prevScan;  // Previous scan
        Pose2D initPose;  // Location of map origin.

        double scthre;    //Score threshold.If it is larger than this,
                          //  it is considered as ICP failure
        double nthre;     //Usage points threshold.If it is smaller than this,
                          //it is considered as ICP failure
        double atd;       //Cumulative mileage
        bool dgcheck;     //Do degeneration processing

        PointCloudMap *pcmap;    //Point cloup map
        RefScanMaker *rsm;       //Generate reference scan
        PoseEstimatorICP *estim; //robot position estimator
        ScanPointResampler *spres;  //Uniform scan point spacing 
        ScanPointAnalyser *spana;  //Scan point normal calculation
        PoseFuser *pfu;           //sensor fusion device
        Eigen::Matrix3d cov;      //Covariance matrix of robot movement
        Eigen::Matrix3d totalCov; //Covariance Matrix of robot position

        std::vector<PoseCov> poseCovs; // for debugging


    public:
        ScanMatcher2D()
            :cnt(-1), scthre(1.0), nthre(50), dgcheck(false), atd(0),pcmap(nullptr),
             estim(nullptr), rsm(nullptr), spres(nullptr), spana(nullptr), pfu(nullptr) {}
        
        ~ScanMatcher2D(){}

        void setPoseEstimator(PoseEstimatorICP *p) { estim = p; }

        void setPoseFuser(PoseFuser *p) { pfu = p;}

        void setRefScanMaker(RefScanMaker *r)
        {
            rsm = r;
            if (pcmap != nullptr)
                rsm->setPointCloudMap(pcmap);
        }

        void setPointCloudMap(PointCloudMap *m)
        {
            pcmap = m;
            if (rsm != nullptr)
                rsm->setPointCloudMap(pcmap);
        }

        void setScanPointResampler(ScanPointResampler *s) { spres = s;}

        void setScanPointAnalyser(ScanPointAnalyser *s) { spana = s; }

        void reset() { cnt = -1;}

        void setInitPose(Pose2D &p) {initPose = p;}

        void setDgCheck(bool t) { dgcheck = t;}

        Eigen::Matrix3d &getCovariance() { return (cov); }

        std::vector<PoseCov> &getPoseCovs() { return (poseCovs);}

        bool matchScan(Scan2D &scan);
        void growMap(const Scan2D &scan, const Pose2D &pose);
};

#endif

