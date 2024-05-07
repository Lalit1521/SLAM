#include "PoseOptimizerSD.h"

using namespace std;

// Under the fixed data correspondence, give the initial value initPose and calculate the estimated robot position estPose
double PoseOptimizerSD::optimizePose(Pose2D &initPose, Pose2D &estPose)
{
    double th = initPose.th;
    double tx = initPose.tx;
    double ty = initPose.ty;
    double txmin = tx, tymin = ty, thmin = th; //Minimum cost solution
    double evmin = HUGE_VAL;                   //Minimum cost
    double evold = evmin;                      //Previous cost value.Used for convergence judgement

    double ev = cfunc->calValue(tx, ty, th);   //cost calculation
    printf("debug_error = %f\n", ev);
    int nn = 0;                   //Number of repetitions
    double kk = 0.00001;          //Steepest descent width factor
    while (abs(evold - ev) > evthre)
    {
    // Convergence judgment. Terminates if the change from the previous value is small
        nn++;
        evold = ev;

        //Partial differentiation by numerical calculation
        double dEtx = (cfunc->calValue(tx + dd, ty, th) - ev)/ dd;
        double dEty = (cfunc->calValue(tx, ty + dd, th) - ev)/ dd;
        double dEth = (cfunc->calValue(tx, ty, th + da) - ev)/ da;

        //Multiply the differential coefficient by kk to get step width
        double dx = -kk * dEtx;
        double dy = -kk * dEty;
        double dth = -kk * dEth;
        tx += dx;
        ty += dy;
        th += dth;  // Add step width to determiner next search position

        ev = cfunc->calValue(tx, ty, th); // Calculate cost at that location

        if (ev < evmin)
        {
            //Update if ev is the minimum so far
            evmin = ev;
            txmin = tx;
            tymin = ty;
            thmin = th;
        }
    }
    printf("pose_optimize_iteration %d\n", nn);

    ++allN;
    if (allN > 0 && evmin < 100)
        sum += evmin;

    estPose.setVal(txmin, tymin, thmin); //Save the solution that gives the minimum value

    return (evmin);
}