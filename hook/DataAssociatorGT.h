#ifndef DATA_ASSOCIATOR_GT_H_
#define DATA_ASSOCIATOR_GT_H_

#include "DataAssociator.h"
#include "NNGridTable.h"

// Use a lattice table to map points between the current and reference scans
class DataAssociatorGT : public DataAssociator
{
private:
    NNGridTable nntab;   //lattice table

public:
    DataAssociatorGT() {}

    ~DataAssociatorGT() {}

    //Use the reference scan point rlps as a pointer and put it in nntab
    virtual void setRefBase(const std::vector<LPoint2D> &rlps)
    {
        nntab.clear();
        for (size_t i = 0; i < rlps.size(); i++)
            nntab.addPoint(&rlps[i]);  //Store as a pointer
    }

    virtual double findCorrespondence(const Scan2D *curScan, const Pose2D &predPose);
};

#endif