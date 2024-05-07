#ifndef _POSEESTIMATOR_ICP_H_
#define _POSEESTIMATOR_ICP_H_

#include "DataAssociator.h"
#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"
#include "PoseOptimizer.h"
#include "Scan2D.h"
#include <vector>

class PoseEstimatorICP
{
    private:
        const Scan2D *curScan;   //scan now
        size_t usedNum;          //Points used for ICP.Use LoopDetector for reliability
        double pnrate;           //Proportion of correctly matched points

        PoseOptimizer *popt;     //optimization class
        DataAssociator *dass;    //data mapping class

    public:
        double totalError;    // Total error
        double totalTime;     // Total processing time

    public:
        PoseEstimatorICP() : usedNum(0), pnrate(0), totalError(0), totalTime(0) {}

        ~PoseEstimatorICP() {}

        void setPoseOptimizer(PoseOptimizer *p) {popt = p;}

        void setDataAssociator(DataAssociator *d) { dass = d;}

        double getPnrate() { return (pnrate);}

        size_t getUsedNum() {return (usedNum);}

        void setScanPair (const Scan2D *c, const Scan2D *r)
        {
            curScan = c;
            dass->setRefBase(r->lps);  //Register reference scan for mapping
        }

        void setScanPair(const Scan2D *c, const std::vector<LPoint2D> &refLps)
        {
            curScan = c;
            dass->setRefBase(refLps);  //Register reference scan points for mapping
        }

        double estimatePose(Pose2D &initPose, Pose2D &estPose);
    
};

#endif