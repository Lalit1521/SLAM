#ifndef _REF_SCAN_MAKER_H_
#define _REF_SCAN_MAKER_H_

#include "LPoint2D.h"
#include "MyUtil.h"
#include "PointCloudMap.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include <vector>

class RefScanMaker
{
    protected:
        const PointCloudMap *pcmap;   //point cloud map
        Scan2D refScan;               //Reference scan body.

    public:
        RefScanMaker() : pcmap(nullptr) {}

        ~RefScanMaker() {}

        void setPointCloudMap(const PointCloudMap *p) { pcmap = p; }

        virtual const Scan2D *makeRefScan() = 0;
};

#endif