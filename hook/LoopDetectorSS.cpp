#include "LoopDetectorSS.h"

using namespace std;

//loop detection
// Find a place from the robot trajectory that is close to the current position curPose and whose shape matches the current scan curScan, and create a pose arc.
bool LoopDetectorSS::detectLoop(Scan2D *curScan, Pose2D &curPose, int cnt)
{
    printf("-- detectLoop--\n");

    //Find the nearest partial map
    double atd = pcmap->atd;       //Current actual cumulative milage
    double atdR = 0;               //Cumulative milage when tracing robot trajectory
    const vector<Submap> &submaps = pcmap->submaps;  //Partial map
    const vector<Pose2D> &poses = pcmap->poses;      //Robot Trajectory
    double dmin = HUGE_VAL;         // Minimum distance to last visited point
    size_t imin = 0, jmin = 0;      // Indexx of previous visited point with minimum distance
    Pose2D prevP;
    for (size_t i = 0; i < submaps.size() - 1; i++)
    {//Search for something other than current partial map
        const Submap &submap = submaps[i];
        for (size_t j = submap.cntS; j<= submap.cntE; j++)
        {// Each robot position on the partial map
            Pose2D p = poses[j]; //robot position
            atdR += sqrt((p.tx - prevP.tx) * (p.tx - prevP.tx) +
                         (p.ty - prevP.ty) * (p.ty - prevP.ty));
            if (atd - atdR < atdthre)
            {// If the distance traveled to the current location is short, it will not be considered a loop and will stop.
                i = submaps.size(); //This line will make us break out of the outer for loop
                break;
            }
            prevP = p;

            double d = (curPose.tx - p.tx) * (curPose.tx - p.tx) +
                       (curPose.ty - p.ty) * (curPose.ty - p.ty);
            if (d < dmin)
            {// Is the distance between the current position and p the smallest so far
                dmin = d;
                imin = i;  //Index of canditate partial map
                jmin = j;  //Last visited point index
            }
        }  
    }

    printf("dmin = %g, radius = %g, imin = %lu, jmin = %lu\n", sqrt(dmin), radius, imin, jmin);

    if (dmin > radius * radius)  //Loop detection is not possible if the distance is less than this threshold
    {
        return(false);
    }

    Submap &refSubmap = pcmap->submaps[imin]; //Make the nearest submap the reference scan
    const Pose2D &initPose = poses[jmin];
    printf("curPose:  tx=%g, ty=%g, th=%g\n", curPose.tx, curPose.ty, curPose.th);
    printf("initPose: tx=%g, ty=%g, th=%g\n", initPose.tx, initPose.ty,
           initPose.th);

    //Find the revisit point location
    Pose2D revisitPose;
    bool flag = estimateRevisitPose(curScan, refSubmap.mps, curPose, revisitPose);

    if (flag)
    {
        Eigen::Matrix3d icpCov;    //ICP Covariance
        double ratio = pfu->calIcpCovariance(revisitPose, curScan,
                                             icpCov); // Calculate covariance of ICP
        
        LoopInfo info;             //Loop detection result
        info.pose = revisitPose;   //Set revisit point position in loop arc information
        info.cov = icpCov;         //Set covariance to loop arc information
        info.curId = cnt;          //Current position node id
        info.refId = static_cast<int>(jmin); //Node ID of last visited point
        makeLoopArc(info);          //Loop arc generation

        Scan2D refScan;
        Pose2D spose = poses[refSubmap.cntS];
        refScan.setSid(info.refId);
        refScan.setLps(refSubmap.mps);
        refScan.setPose(spose);
        LoopMatch lm(*curScan, refScan, info);
        loopMatches.emplace_back(lm);
        printf("curId = %d, refId = %d\n", info.curId, info.refId);
    }

    return (flag);

}



// Perform ICP using the current scan curScan and the point cloud refLps of the partial map to find the position of the revisited point.
bool LoopDetectorSS::estimateRevisitPose(const Scan2D *curScan,
                                         const vector<LPoint2D> &refLps,
                                         const Pose2D &initPose,
                                         Pose2D &revisitPose)
{
    dass->setRefBase(refLps);   //Set reference point cloud to data mathcer
    cfunc->setEvlimit(0.2);    //Cost function error threshold

    size_t usedNumMin = 50;
    // Thoroughly examine the area around the initial position initPose.
    //For efficiency, we do not perform ICP and simply check the matching score at each position.

    double rangeT = 1;  //Translational search range [m]
    double rangeA = 45; //Rotation search range[degrees]
    double dd = 0.2;    //Translational search interval
    double da = 2;      //Rotation search interval [degrees]
    double pnrateMax = 0;
    vector<double> pnrates;
    double scoreMin = 1000;
    vector<double> scores;
    vector<Pose2D> candidates; //Candidate position with good scores
    
    for (double dy = -rangeT; dy <= rangeT; dy += dd)
    {// Repeated search for translation y
        double y = initPose.ty + dy;  // Add displacement dy to initial position
        for (double dx = -rangeT; dx <= rangeT; dx += dd)
        { // Repeated search for translation x
            double x = initPose.tx + dx; // Add displacement dx to initial position
            for (double dth = -rangeA; dth <= rangeA; dth += da)
            { // Rotation search repetition
                double th = MyUtil::add(initPose.th, dth); // Add displacement dth to initial position
                Pose2D pose(x, y, th);
                double mratio = dass->findCorrespondence(curScan, pose);  // Data correspondence by position pose
                size_t usedNum = dass->curLps.size();
                if (usedNum < usedNumMin || mratio < 0.9) // Skip if response rate is poor
                    continue;
                cfunc->setPoints(dass->curLps, dass->refLps);  // Set point cloud to cost function
                double score = cfunc->calValue(x, y, th);  //Cost Value(matching score)
                double pnrate = cfunc->getPnrate(); // Correspondence rate of points

                if (pnrate > 0.8)
                {
                    candidates.emplace_back(pose);
                    if (score < scoreMin)
                        scoreMin = score;
                    scores.push_back(score);
                }
            }
        }
    }
    printf("candidates.size=%lu\n", candidates.size());
    if (candidates.size() == 0)
        return (false);

    // Select the best position from among the candidate positions using ICP
    Pose2D best;  //Best Candidate
    double smin = 1000000;    //ICP score minimum
    estim->setScanPair(curScan, refLps);
    for (size_t i = 0; i < candidates.size(); i++)
    {
        Pose2D p = candidates[i];     //Canditate position
        printf("score = %g\n", scores[i]);
        Pose2D estP;
        double score = estim->estimatePose(p, estP); // Find matching position with ICP 
        double pnrate = estim->getPnrate();   //Correspondence rate of points with less error
        size_t usedNum = estim->getUsedNum(); //Points used in ICP
        if (score < smin && pnrate >= 0.9 && usedNum >= usedNumMin)
        { //Loop detection has strict conditions
            smin = score;
            best = estP;
            printf("smin = %g , pnrate = %g, usedNum = %lu\n", smin, pnrate, usedNum);
        }
    }

    //Found if minimum score is less than threshold
    if (smin <= scthre)
    {
        revisitPose = best;
        return (true);
    }

    return (false);
}

// Generate a loop arc with the previous visited point (refId) as the start node and the current position (curId) as the end node.
void LoopDetectorSS::makeLoopArc(LoopInfo &info)
{
    if (info.arcked) //info's arc is already set
        return;
    info.setArcked(true);

    Pose2D srcPose = pcmap->poses[info.refId]; // Location of previous visited point
    Pose2D dstPose(info.pose.tx, info.pose.ty, info.pose.th);// Location of revisit point
    Pose2D relPose;
    Pose2D::calRelativePose(dstPose, srcPose, relPose);  //Loop arc constraint

    // Since the arc constraint is a relative position from the start node, convert the covariance to the loop arc start node coordinate system.
    Eigen::Matrix3d cov;
    CovarianceCalculator::rotateCovariance(srcPose, info.cov, cov, true);

    PoseArc *arc = pg->makeArc(info.refId, info.curId, relPose, cov); // Loop arc generation
    pg->addArc(arc); 
    printf("makeLoopArc: pose arc added\n");
    printf("srcPose: tx=%g, ty=%g, th=%g\n", srcPose.tx, srcPose.ty, srcPose.th);
    printf("dstPose: tx=%g, ty=%g, th=%g\n", dstPose.tx, dstPose.ty, dstPose.th);
    printf("relPose: tx=%g, ty=%g, th=%g\n", relPose.tx, relPose.ty, relPose.th);
    PoseNode *src = pg->findNode(info.refId);
    PoseNode *dst = pg->findNode(info.curId);
    Pose2D relPose2;
    Pose2D::calRelativePose(dst->pose, src->pose, relPose2);
    printf("relPose2: tx=%g, ty=%g, th=%g\n", relPose2.tx, relPose2.ty,
           relPose2.th);
}