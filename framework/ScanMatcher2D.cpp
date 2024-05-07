#include "ScanMatcher2D.h"

using namespace std;

bool ScanMatcher2D::matchScan(Scan2D &curScan)
{
    ++cnt;

    printf("---- ScanMatcher2D: cnt=%d start ----\n", cnt);
    printf("before resample size = %lu\n", curScan.lps.size());
    //If spres is set, equalize the scan point spacing
    if (spres != nullptr)
        spres->resamplePoints(&curScan);

    // If spana is set, calculate the normal of the scan point
    if (spana != nullptr)
        spana->analysePoints(curScan.lps);

    //The first scan is simply mapped
    if (cnt == 0)
    {
        growMap(curScan, initPose);
        prevScan = curScan;
        return (true);      
    }

    // Calculate the amount of movement using the odometry value contained in Scan
    Pose2D odoMotion;     //Travel amount on odometry
    Pose2D::calRelativePose(curScan.pose, prevScan.pose,
                            odoMotion); //The relative position to the previous
                                        // scan is the amount of movement
    
    Pose2D lastPose = pcmap->getLastPose(); //Just before position
    Pose2D predPose;                        //Predicted position by odometry
    Pose2D::calGlobalPose(odoMotion, lastPose,
                          predPose);     //Add the movement amount to the previous
                                         //position to get the predicted pose

    const Scan2D *refScan = rsm->makeRefScan();  // Generate reference scan
    estim->setScanPair(&curScan, refScan);       // set scan to ICP
    printf("curscan.size = %lu, refScan.size = %lu\n", curScan.lps.size(),
            refScan->lps.size());

    Pose2D estPose;    //Estimated position by ICP
    double score = estim->estimatePose(predPose, 
                            estPose); //Execute ICP using predicted pose as initial guess
    size_t usedNum = estim->getUsedNum();

    bool successful;    //Whether scan matching was successful
    if (score <= scthre && usedNum >= nthre)
        successful = true;
    else
        successful = false;
    printf("score=%g, usedNum=%lu, successful%d\n", score, usedNum, successful);
    if (dgcheck)
    { //When dealing with degeneration
        if (successful)
        {
            Pose2D fusedPose;          //Fusion result
            Eigen::Matrix3d fusedCov;  //Covariance after sensor fusion

            pfu->setRefScan(refScan);
            // The sensor fusion device pfu fuses ICP results and odometry value
            
            double ratio = pfu->fusePose(&curScan, estPose, odoMotion, lastPose,
                                         fusedPose, fusedCov);
            
            estPose = fusedPose;
            cov = fusedCov;

            printf("ratio=%g. Pose fused.\n", ratio);  //ratio is the degree of degeneration.

            //Accumulate covariance
            //Eigen::Matrix3d covL;    //Covariance of movement amount
            //CovarianceCalculator::rotateCovariance(lastPose, fusedCov,
            //                                     covL, true);//Covert to covariance of movement amount
            //Eigen::Matrix3d tcov;    //Covariance after accumulation
            //CovarianceCalculator::accumulateCovariance(lastPose, estPose, totalCov,
            //                                          covL, tcov);
            //totalCov = tcov;
        }
        else
        {   //If ICP is not successful, use predicted pose by odometry
            estPose = predPose;
            pfu->calOdometryCovariance(odoMotion, lastPose,
                                        cov);  //Cov is just odometry covariance

        }
    }
    else
    {
        if (!successful)
            estPose = predPose;
    }
    growMap(curScan, estPose);   // Add scan point cloud to map
    prevScan = curScan;          // Last minute scan settings

    printf("predPose: tx=%g, ty=%g, th=%g\n", predPose.tx, predPose.ty,
           predPose.th); 
    printf("estPose: tx=%g, ty=%g, th=%g\n", estPose.tx, estPose.ty, estPose.th);
    //printf("cov: %g, %g, %g, %g\n", totalCov(0, 0), totalCov(0, 1),
    //       totalCov(1, 0), totalCov(1, 1));
    //printf("mcov: %g, %g, %g, %g\n", pfu->mcov(0, 0), pfu->mcov(0, 1),
    //       pfu->mcov(1, 0), pfu->mcov(1, 1));
    //printf("ecov: %g, %g, %g, %g\n", pfu->ecov(0, 0), pfu->ecov(0, 1),
    //       pfu->ecov(1, 0), pfu->ecov(1, 1));

    //PoseCov pcov(estPose, pfu->ecov);
    //poseCovs.emplace_back(pcov);

    Pose2D estMotion; // Estimated travel amount
    Pose2D::calRelativePose(estPose, lastPose, estMotion);
    atd += sqrt(estMotion.tx * estMotion.tx + estMotion.ty * estMotion.ty);
    printf("atd=%g\n", atd);

    return (successful);
}

//Add scans now to grow the map
void ScanMatcher2D::growMap(const Scan2D &scan, const Pose2D &pose)
{
    const vector<LPoint2D> &lps = scan.lps;
    const double(*R)[2] = pose.Rmat;   //Scan point cloud (robot coordinate system)
    double tx = pose.tx;
    double ty = pose.ty;

    vector<LPoint2D> scanG;  //Point cloud in map coordinate system
    for (size_t i = 0; i < lps.size(); i++)
    {
        const LPoint2D &lp = lps[i];
        if (lp.type == ISOLATE)   // Exclude isolated points (no normal)
            continue;
        double x = R[0][0] * lp.x + R[0][1] * lp.y + tx;  //convert to map co-ordinate
        double y = R[1][0] * lp.x + R[1][1] * lp.y + ty;
        double nx = R[0][0] * lp.nx + R[0][1] * lp.ny; 
        double ny = R[1][0] * lp.nx + R[1][1] * lp.ny;   

        LPoint2D mlp(cnt, x, y);  //Generate new point
        mlp.setNormal(nx,ny);
        mlp.setType(lp.type);
        scanG.emplace_back(mlp);   // mlp is copied into vector
    }

    //Register on point cloud map pcmap
    pcmap->addPose(pose);
    pcmap->addPoints(scanG);
    pcmap->setLastPose(pose);
    pcmap->setLastScan(scan);      //Save for reference scan
    pcmap->makeLocalMap();         //Generate local map
    pcmap->makeOccupancyGridMap();

    printf("ScanMatcher: estPose: tx=%g, ty=%g, th=%g\n", pose.tx, pose.ty,
           pose.th); // For confirmation
    
}