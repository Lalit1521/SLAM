#include "PoseFuser.h"

using namespace std;

// Combining ICP and odometry estimated movement in sequential SLAM. Have a reference scan in dass. The covariance matrix of the amount of movement is stored in cov.
double PoseFuser::fusePose(Scan2D *curScan, const Pose2D &estPose,
                           const Pose2D &odoMotion, const Pose2D &lastPose,
                           Pose2D &fusedPose, Eigen::Matrix3d &fusedCov)
{
    //ICP Covariance
    dass->findCorrespondence(curScan, estPose);// Corresponding between current scan point group and reference scan point group using estimated position estPose
    double ratio = cvc.calIcpCovariance(
        estPose, dass->curLps, dass->refLps,
        ecov);

    // Odometry location and covariance. When using the velocity motion model, the covariance is too small in a short period of time, so we calculate a larger value using the simplified version.

    Pose2D predPose;
    Pose2D::calGlobalPose(odoMotion, lastPose,
                          predPose); // Calculate the predicted position by adding the movement amount to the previous position lastPose
    
    Eigen::Matrix3d mcovL;
    double dT = 0.1;
    cvc.calMotionCovarianceSimple(
        odoMotion, dT, mcovL); // Covariance of movement amount obtained by odometry (simplified version)
    CovarianceCalculator::rotateCovariance(estPose, mcovL,
                mcov); // Rotate at current position estPose to obtain covariance mcov in map coordinate system
    
    // Both ecov, mcov, and cov are values in the local coordinate system with the origin at lastPose.
    Eigen::Vector3d mu1(estPose.tx, estPose.ty,
                        DEG2RAD(estPose.th)); //Estimated value by ICP

    Eigen::Vector3d mu2(predPose.tx, predPose.ty,
                        DEG2RAD(predPose.th)); //Estimated value by odometry

    Eigen::Vector3d mu;
    fuse(mu1, ecov, mu2, mcov, mu, fusedCov);  //Fusion of two normal distribution

    fusedPose.setVal(mu[0], mu[1], RAD2DEG(mu[2])); // Store the fused movement amount

    //totalCov = fusedCov;
     // For confirmation
    printf("fusePose\n");
    double vals[2], vec1[2], vec2[2];
    //printf("ecov: det=%g, ", ecov.determinant());
    cvc.calEigen(ecov, vals, vec1, vec2);
    //printf("mcov: det=%g, ", mcov.determinant());
    cvc.calEigen(mcov, vals, vec1, vec2);
    //printf("fusedCov: det=%g, ", fusedCov.determinant());
    cvc.calEigen(fusedCov, vals, vec1, vec2);

    //printf("predPose: tx=%g, ty=%g, th=%g\n", predPose.tx, predPose.ty,
    //       predPose.th);
    //printf("estPose: tx=%g, ty=%g, th=%g\n", estPose.tx, estPose.ty, estPose.th);
    //printf("fusedPose: tx=%g, ty=%g, th=%g\n", fusedPose.tx, fusedPose.ty,
    //       fusedPose.th);

    return (ratio);
}


// Fuse two normal distributions. mu is the mean and cv is the covariance.
double PoseFuser::fuse(const Eigen::Vector3d &mu1, const Eigen::Matrix3d &cv1,
                        const Eigen::Vector3d &mu2, const Eigen::Matrix3d &cv2,
                        Eigen::Vector3d &mu, Eigen::Matrix3d &cv)
{
    //Fusion of covariance matrices
    Eigen::Matrix3d IC1 = MyUtil::svdInverse(cv1);
    Eigen::Matrix3d IC2 = MyUtil::svdInverse(cv2);
    Eigen::Matrix3d IC = IC1 + IC2;
    cv = MyUtil::svdInverse(IC);

    //Angle correction.To maintain continuity when merging.
    Eigen::Vector3d mu11 = mu1;  //Align ICP direction to odometry
    double da = mu2(2) - mu1(2);
    if (da > M_PI)
        mu11(2) += 2 * M_PI;
    else if (da < -M_PI)
        mu11(2) -= 2 * M_PI;

    //fusion of averages
    Eigen::Vector3d nu1 = IC1 * mu11;
    Eigen::Vector3d nu2 = IC2 * mu2;
    Eigen::Vector3d nu3 = nu1 + nu2;
    mu = cv * nu3;
    
    /// Angle correction. fit in (-pi, pi)
    if (mu(2) > M_PI)
        mu(2) -= 2 * M_PI;
    else if (mu(2) < -M_PI)
        mu(2) += 2 * M_PI;

    //Calculation of coeffecient part
    Eigen::Vector3d W1 = IC1 * mu11;
    Eigen::Vector3d W2 = IC2 * mu2;
    Eigen::Vector3d W = IC * mu;
    double A1 = mu1.dot(W1);
    double A2 = mu2.dot(W2);
    double A = mu.dot(W);
    double K = A1 + A2 - A;

    return (K);
}

void PoseFuser::calOdometryCovariance(const Pose2D &odoMotion,
                                      const Pose2D &lastPose,
                                      Eigen::Matrix3d &mcov)
{
    Eigen::Matrix3d mcovL;
    double dT = 0.1;
    cvc.calMotionCovarianceSimple(
        odoMotion, dT, mcovL); // Covariance of movement amount obtained by odometry (simplified version)
    CovarianceCalculator::rotateCovariance(
        lastPose, mcovL,
        mcov); // Rotate at the previous position lastPose to obtain the position covariance mcov
}

void PoseFuser::printMatrix(const Eigen::Matrix3d &mat)
{
    for (int i = 0; i < 3; i++)
        printf("%g %g %g\n", mat(i, 0), mat(i, 1), mat(i, 2));
}