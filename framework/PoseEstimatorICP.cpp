#include "PoseEstimatorICP.h"
#include <boost/timer.hpp>

using namespace std;

double PoseEstimatorICP::estimatePose(Pose2D &initPose, Pose2D &estPose)
{
    boost::timer tim;

    double evmin = HUGE_VAL;  //Minimum cost.Initial value is large
    double evthre = 0.000001; //Cost change threshold.If the amount of change is
                              //less than this, the repetition ends
    popt->setEvthre(evthre);
    popt->setEvlimit(0.2);    //evlimit is the outlier threshold[m]

    double ev = 0;            //cost
    double evold = evmin;     //Previos value. Used for convergence judgement
    Pose2D pose = initPose;
    Pose2D poseMin = initPose;
    for (int i = 0; abs(evold - ev) > evthre && i <100; i++)
    {
        if (i > 0)
            evold = ev;
        double mratio = dass->findCorrespondence(curScan, pose);  //data mapping
        //printf("mratio = %f\n ", mratio);
        Pose2D newPose;
        popt->setPoints(dass->curLps, dass->refLps);  //Pass the corresponding results
        ev = popt->optimizePose(pose, newPose);   //Optimization of robot position in that correspondence
        pose = newPose;

        if (ev < evmin)
        {
            poseMin = newPose;
            evmin = ev;
        }
        //printf("dass.curLps.size=%lu, dass.refLps.size=%lu\n", dass->curLps.size(), dass->refLps.size());
        //printf("mratio=%g\n", mratio);
        //printf("i=%d: ev=%g, evold=%g\n", i, ev, evold);
    }

    pnrate = popt->getPnrate();
    usedNum = dass->curLps.size();

    estPose = poseMin;

    //printf("finalError=%g, pnrate=%g\n", evmin, pnrate);
    //printf("estPose:  tx=%g, ty=%g, th=%g\n", pose.tx, pose.ty,
    //       pose.th); // For confirmation

    double t1 = 1000 * tim.elapsed();
    //printf("PoseEstimatorICP: t1=%g\n", t1); // processing time

    if (evmin < HUGE_VAL)
        totalError += evmin;    //Total error
    totalTime += t1;            //Total processing time
    //printf("totalError=%g, totalTime=%g\n", totalError,
    //       totalTime); // For confirmation

        return (evmin);
}