#ifndef DATA_ASSOCIATOR_LS_H_
#define DATA_ASSOCIATOR_LS_H_

#include "DataAssociator.h"

// Match points between current and reference scans using linear search
class DataAssociatorLS: public DataAssociator
{
    private:
        std::vector<const LPoint2D *> baseLps; //Store reference scan points.For work

    public:
        DataAssociatorLS() {}

        ~DataAssociatorLS() {}

        //Set reference scan point rlps as a pointer to baseLps
        virtual void setRefBase(const std::vector<LPoint2D> &rlps)
        {
            baseLps.clear();
            for (size_t i = 0; i < rlps.size(); i++)
            {
                baseLps.push_back(&rlps[i]);  // Store as a pointer
            }    
        }

        virtual double findCorrespondence(const Scan2D *curScan,
                                          const Pose2D &predPose);
};

#endif
