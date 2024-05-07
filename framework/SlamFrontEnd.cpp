#include "SlamFrontEnd.h"

using namespace std;

void SlamFrontEnd::init()
{
    smat->reset();
    smat->setPointCloudMap(pcmap);
    sback.setPointCloudMap(pcmap);
    path.setPointCloudMap(pcmap);
    motion.setPointCloudMap(pcmap);
}

void SlamFrontEnd::process(Scan2D &scan)
{
    if (cnt == 0)
        init();

    //Scan matching
    smat->matchScan(scan);

    Pose2D curPose = pcmap->getLastPose(); //This robot current pose

    if (cnt == 0)
    { //At first time step, place a node
        pg->addNode(curPose);
    }
    else
    {//Add edge (relative pose and inverse(covariance_matrix))
        Eigen::Matrix3d &cov = smat->getCovariance();
        makeOdometryArc(curPose, cov);
    }

    if (cnt % keyframeSkip == 0)
    {
        if (cnt == 0)
            pcmap->setNthre(1);
        else    
            pcmap->setNthre(5);
        pcmap->makeGlobalMap();
    }

    //loop closure
    if (cnt > keyframeSkip && cnt % keyframeSkip == 0)
    {
        bool flag = lpd->detectLoop(&scan, curPose, cnt);
        if (flag)
        {
            sback.adjustPoses();  //Do pose graph optimization when loop closure is found
            sback.remakeMaps();   //Correction of maps and pose graphs
            //path.loopclosurePlan();
        }
    }

    printf("pcmap.size = %lu\n", pcmap->globalMap.size()); 
    
    countLoopArcs();

    //PathPlanning
    if (cnt > -1)
    {
        //path.planning();
        Pose2D Goal(-2.0, -2.0, 0.0);
        path.planToGoal(Goal);
    }

    //Motion Planning
    if (cnt > -1)
    {
        motion.command();
    }
    ++cnt;


}

bool SlamFrontEnd::makeOdometryArc(Pose2D &curPose,
                                    const Eigen::Matrix3d &fusedCov)
{
    if (pg->nodes.size() == 0)    //For the first node just retuen without making edge
        return (false);
    PoseNode *lastNode = pg->nodes.back();  //Add the last Node
    PoseNode *curNode = pg->addNode(curPose);  

    //Calculate the relative pose for the edge
    Pose2D &lastPose = lastNode->pose;
    Pose2D relPose;
    Pose2D::calRelativePose(curPose, lastPose, relPose);
    printf("sfront: lastPose: tx=%g, ty=%g, th=%g\n", lastPose.tx, lastPose.ty, lastPose.th);

    Eigen::Matrix3d cov;
    CovarianceCalculator::rotateCovariance(lastPose, fusedCov, cov,
                                            true); //Convert fusedCov to odometry Motion

    PoseArc *arc = pg->makeArc(lastNode->nid, curNode->nid, relPose, cov);
    pg->addArc(arc);   

    return (true);
}

void SlamFrontEnd::countLoopArcs()
{
    vector<PoseArc *> &parcs = pg->arcs;
    int an = 0;
    for (size_t i = 0; i < parcs.size(); i++)
    {
        PoseArc *a = parcs[i];
        PoseNode *src = a->src;
        PoseNode *dst = a->dst;
        if (src->nid != dst->nid - 1)
            ++an;
    }
    printf("loopArcs.size=%d\n", an);
}