#ifndef DATA_ASSOCIATOR_H_
#define DATA_ASSOCIATOR_H_

#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include <vector>

class DataAssociator
{
    public:
        std::vector<const LPoint2D *> curLps;  //Corresponding point cloud of current scan
        std::vector<const LPoint2D *> refLps;  //Point cloud of matched reference scans

        DataAssociator() {}

        ~DataAssociator() {}

        virtual void setRefBase(const std::vector<LPoint2D> &lps) = 0;
        virtual double findCorrespondence(const Scan2D *curScan,
                                        const Pose2D &predPose) = 0;

};

#endif