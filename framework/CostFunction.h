#ifndef _COST_FUNCTION_H_
#define _COST_FUNCTION_H_

#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include <vector>

class CostFunction
{
    protected:
        std::vector<const LPoint2D *> curLps; // Corresponding point cloud of current scan
        std::vector<const LPoint2D *> refLps; // Point cloud of matched reference scan
        double evlimit;                       // Distance threshold for determining correspondence in matching
        double pnrate;                        //Ratio of points with matching error within evlimit

    public:
        CostFunction() : evlimit(0), pnrate(0) {}

        ~CostFunction() {}

        void setEvlimit(double e) { evlimit = e; }

        // Set corresponding point cloud cur, ref with DataAssociator
        void setPoints(std::vector<const LPoint2D *> &cur,
                       std::vector<const LPoint2D *> &ref)
        {
            curLps = cur;
            refLps = ref;
        }

        double getPnrate() { return (pnrate); }

        virtual double calValue(double tx, double ty, double th) = 0;
};

#endif