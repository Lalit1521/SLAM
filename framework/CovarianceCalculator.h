#ifndef COVARIANCE_CALCULATOR_H_
#define COVARIANCE_CALCULATOR_H_

#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"
#include <vector>

// Calculate the covariance of the estimates by ICP and the covariance of the estimates by odometry
class CovarianceCalculator
{
private:
    double dd;  //step of numerical differentiation
    double da;  //step of numerical differentiation
    double a1;  //Coefficient of odometry covariance
    double a2;  //Coefficient of odometry covariance

public:
    CovarianceCalculator() : dd(0.00001), da(0.00001) {}

    ~CovarianceCalculator() {}

    void setAlpha(double a1_, double a2_)
    {
        a1 = a1_;
        a2 = a2_;
    }

    double calIcpCovariance(const Pose2D &pose, std::vector<const LPoint2D *> &curLps,
                             std::vector<const LPoint2D *> &refLps, Eigen::Matrix3d &cov);

    double calPDistance(const LPoint2D *clp, const LPoint2D *rlp, double tx,
                        double ty, double th);

    void calMotionCovarianceSimple(const Pose2D &motion, double dT, Eigen::Matrix3d &cov);

    void calMotionCovariance(double th, double tx, double dy, double dth,
                             double dt, Eigen::Matrix3d &covx, bool accum = false);

    void calUk(double vt, double wt, Eigen::Matrix2d &Uk);
    void calJxk(double th, double vt, double dt, Eigen::Matrix3d &Jxk);
    void calJuk(double th, double dt, Eigen::Matrix<double, 3, 2> &Juk);

    double calEigen(const Eigen::Matrix3d &cov, double *vals, double *vec1,
                    double *vec2);

    static void accumulateCovariance(const Pose2D &curPose, const Pose2D &prevPose,
                                     const Eigen::Matrix3d &prevCov, 
                                     const Eigen::Matrix3d &mcov,
                                     Eigen::Matrix3d &curCov);

    static void rotateCovariance(const Pose2D &pose, const Eigen::Matrix3d &cov,
                                 Eigen::Matrix3d &icov, bool reverse = false);

};

#endif