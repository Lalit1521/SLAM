#ifndef P2O_DRIVER2D_H_
#define P2O_DRIVER2D_H_

#include <fstream>
#include <iostream>
#include <vector>

#include "MyUtil.h"
#include "Pose2D.h"
#include "PoseGraph.h"
#include "Scan2D.h"

class P2oDriver2D
{
public:
    P2oDriver2D() {}

    ~P2oDriver2D() {}

    void doP2o(PoseGraph &graph, std::vector<Pose2D> &newPoses, int N);
};

#endif