#ifndef POSE_FUSER_H_
#define POSE_FUSER_H_

#include "CovarianceCalculator.h"
#include "DataAssociator.h"
#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"
#include "Scan2D.h"

//Sensor fusion device.Fuse ICP and odometry estimates
class PoseFuser
{
public:
    Eigen::Matrix3d ecov;  //Fuse ICP and odometry estimates
    Eigen::Matrix3d mcov;  //Odometry covariance matrix
    Eigen::Matrix3d totalCov;

    DataAssociator *dass;  //data matcher
    CovarianceCalculator cvc; //covariance calculator

public:
    PoseFuser() {}
    
    ~PoseFuser() {}

    void setDataAssociator(DataAssociator *d) { dass = d; }

    void setRefScan(const Scan2D *refScan) { dass->setRefBase(refScan->lps); }

    void setRefLps(const std::vector<LPoint2D> &refLps)
    {
        dass->setRefBase(refLps);
    }

    // Calculating the covariance matrix of ICP. What to do after setRefLps.
    double calIcpCovariance(const Pose2D &estMotion, const Scan2D *curScan,
                            Eigen::Matrix3d &cov)
    {
        dass->findCorrespondence(curScan, estMotion);

        //Covariance of ICP.Here we get covariance in world coordinate system
        double ratio = 
            cvc.calIcpCovariance(estMotion, dass->curLps, dass->refLps, cov);

        return (ratio);
    }

    double fusePose(Scan2D *curScan, const Pose2D &estPose,
                    const Pose2D &odoMotion, const Pose2D &lastPose,
                    Pose2D &fusedPose, Eigen::Matrix3d &cov);

    void calOdometryCovariance(const Pose2D &odoMotion, const Pose2D &lastPose,
                               Eigen::Matrix3d &mcov);

    double fuse(const Eigen::Vector3d &mu1, const Eigen::Matrix3d &cv1,
                const Eigen::Vector3d &mu2, const Eigen::Matrix3d &cv2,
                Eigen::Vector3d &mu, Eigen::Matrix3d &cv);

    void printMatrix(const Eigen::Matrix3d &mat);
};

#endif