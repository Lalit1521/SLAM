#include "PoseOptimizerSL.h"
#include <boost/math/tools/minima.hpp>

using namespace std;

double PoseOptimizerSL::optimizePose(Pose2D &initPose, Pose2D &estPose)
{
    double th = initPose.th;
    double tx = initPose.tx;
    double ty = initPose.ty;
    double txmin = tx, tymin = ty, thmin = th;    //Minimum cost solution
    double evmin = HUGE_VAL;                      //Minimum cost
    double evold = evmin;                         //Previous cost value
    Pose2D pose,dir;
    int nn = 0;

    double ev = cfunc->calValue(tx, ty, th);      //cost calculation
    //printf("debug_error = %f\n", ev);
    while (abs(evold - ev) > evthre)
    {
        nn++;
        evold = ev;

        //Partial differentiation calculation
        double dx = (cfunc->calValue(tx + dd, ty, th) - ev)/dd;
        double dy = (cfunc->calValue(tx, ty + dd, th) - ev)/dd;
        double dth = (cfunc->calValue(tx, ty, th + da) - ev)/da;
        tx = tx + dx;
        ty = ty + dy;
        th = th + dth;

        pose.tx = tx;
        pose.ty = ty;
        pose.th = th;    //Search starting point
        //printf("Pose: x - %g, y - %g, th - %g\n", tx, ty, th);

        dir.tx = dx;
        dir.ty = dy;
        dir.th = dth;    //Search direction
        //printf("Direction: x - %g, y - %g, th - %g\n", dx, dy, dth);

        search(ev, pose, dir);  //Line search algorithm
        tx = pose.tx;
        ty = pose.ty;
        th = pose.th;

        ev = cfunc->calValue(tx, ty, th);

        if (ev < evmin)
        {
            //Update if cost is lowest so far
            evmin = ev;
            txmin = tx;
            tymin = ty;
            thmin = th;
        }
    }
    //printf("pose_optimize_iteration %d\n", nn);
    estPose.setVal(txmin, tymin, thmin); //Save the solution to estPose

    return (evmin);
}

double PoseOptimizerSL::search(double ev0, Pose2D &pose, Pose2D &dp)
{
    int bits = numeric_limits<double>::digits;     //Search accuracy
    boost::uintmax_t maxIter = 40;                 //Maximum number of repitation
    pair<double, double> result = boost::math::tools::brent_find_minima(
        [this, &pose, &dp](double tt)
        {return (objFunc(tt, pose, dp));}, -2.0, 2.0, bits, maxIter); //Search range [-2.0 to 2.0]

    double t = result.first;    //Desired step width
    double v = result.second;   //Minimum value

    //printf("root: %g\n", t);

    pose.tx = pose.tx + t * dp.tx;  //Store the minimum required value in pose
    pose.ty = pose.ty + t * dp.ty;
    pose.th = MyUtil::add(pose.th, t * dp.th);

    return (v);
}

//Objective function for line search.
double PoseOptimizerSL::objFunc(double tt, Pose2D &pose, Pose2D &dp)
{
    double tx = pose.tx + tt * dp.tx;
    double ty = pose.ty + tt * dp.ty;
    double th = MyUtil::add(pose.th, tt * dp.th);
    double v = cfunc->calValue(tx, ty, th);     //Cost function value
    //printf("brent v - %g, tt - %g\n", v, tt);

    return (v); 
}