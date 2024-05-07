#include "CovarianceCalculator.h"

using namespace std;
// Find the covariance cov of the estimated robot position by ICP.
// Estimated position pose, current scan point group curLps, reference scan point group refLps
double CovarianceCalculator::calIcpCovariance(
    const Pose2D &pose, std::vector<const LPoint2D *> &curlps,
    std::vector<const LPoint2D *> &refLps, Eigen::Matrix3d &cov)
{
    double tx = pose.tx;
    double ty = pose.ty;
    double th = pose.th;
    double a = DEG2RAD(th);
    vector<double> Jx;  //Column of x in Jacobian matrix
    vector<double> Jy;  //Column of y in Jacobian matrix
    vector<double> Jt;  //th column of Jacobian matrix

    for (size_t i = 0; i < curlps.size(); i++)
    {
        const LPoint2D *clp = curlps[i];   //Current scan point
        const LPoint2D *rlp = refLps[i];   //Reference scan points

        if (rlp->type == ISOLATE)  //Exclude isolated points
            continue;

        double pd0 = calPDistance(clp, rlp, tx, ty, a);  //Cost function value
        double pdx = calPDistance(clp, rlp, tx + dd, ty, a); //Cost function value with slight change in tx
        double pdy = calPDistance(clp, rlp, tx, ty + dd, a); //Cost function value with slight change in ty
        double pdt = calPDistance(clp, rlp, tx, ty, a + da); //Cost function value with slight change in a

        Jx.push_back((pdx - pd0) / dd); //Partial differential (x component)
        Jy.push_back((pdy - pd0) / dd); //Partial differential (y component)
        Jt.push_back((pdt - pd0) / da); //Partial differential (th component)        
    }

    //Calculating the Hessian matrix approximation Jtranspose*J
    Eigen::Matrix3d hes = Eigen::Matrix3d::Zero(3,3); //Approximate Hessian. Initialized to zero

    for (size_t i = 0; i < Jx.size(); i++)
    {
        hes(0, 0) += Jx[i] * Jx[i];
        hes(0, 1) += Jx[i] * Jy[i];
        hes(0, 2) += Jx[i] * Jt[i];
        hes(1, 1) += Jy[i] * Jy[i];
        hes(1, 2) += Jy[i] * Jt[i];
        hes(2, 2) += Jt[i] * Jt[i];
    }
    hes(1, 0) = hes(0, 1);
    hes(2, 0) = hes(0, 2);
    hes(2, 1) = hes(1, 2);

    //The covariance matrix is (approximately) the inverse of Hessian matrix
    //cov = hes.inverse();

    cov = MyUtil::svdInverse(hes); //It is slightly better to use SVD

    double vals[2], vec1[2], vec2[2];
    double ratio =
            calEigen(cov, vals, vec1, vec2);// Calculate the eigenvalues and check the degree of degeneration
    // Adjust the scale of the covariance matrix if necessary    
    //   double kk = 1;     // In case of extreme deviation due to degeneration
    double kk = 0.1; // usually
    cov *= kk;

    return (ratio);
}

//Observation model equation using vertical distance
double CovarianceCalculator::calPDistance(const LPoint2D *clp,
                                          const LPoint2D *rlp, double tx,
                                        double ty, double th)
{
    double x = cos(th) * clp->x - sin(th) * clp->y + tx; // Coordinate transformation of clp using estimated position
    double y = sin(th) * clp->x + cos(th) * clp->y + ty;
    double pdis = (x - rlp->x) * rlp-> nx +
                  (y - rlp->y) * rlp->ny; //Vertical distance from transformed point to rlp

    return (pdis);
}

void CovarianceCalculator::calMotionCovarianceSimple(const Pose2D &motion,
                                                     double dT, Eigen::Matrix3d &cov)
{
    double dis = sqrt(motion.tx * motion.tx + motion.ty * motion.ty);//Moving distance
    double vt = dis/dT;   //Translation speed(m/s)
    double wt = DEG2RAD(motion.th)/dT;   //Angular velocity
    double vthre = 0.02;    //Lower limit value of vt
    double wthre = 0.05;    //Lower limit value of wt

    if (vt < vthre)
        vt = vthre;
    if (wt < wthre)
        wt = wthre;

    double dx = vt;
    double dy = vt;
    double da = wt;

    Eigen::Matrix3d C1;
    C1.setZero();           //Insert only diagonal elements
    C1(0, 0) = 0.001 * dx * dx; // translation component x
    C1(1, 1) = 0.005 * dy * dy; // translation component y
    C1(2, 2) = 0.05 * da * da; // rotational component

    // double kk = 100;
    double kk = 1;
    cov = kk * C1;

    //For confirmation
    printf("calMotionCovarianceSimple\n");
    printf("vt=%g, wt=%g\n", vt, wt);
    double vals[2], vec1[2], vec2[2];
    calEigen(cov, vals, vec1, vec2);
    printf("cov : %g %g %g %g %g %g\n", cov(0, 0), cov(0, 1), cov(0, 2),
           cov(1, 1), cov(1, 2), cov(2, 2));
}

//Error due to running one frame. dT is the time of one frame.
void CovarianceCalculator::calMotionCovariance(double th, double dx, double dy,
                                               double dth, double dt, Eigen::Matrix3d &cov,
                                               bool accum)
{
    setAlpha(1, 5);
    double dis = sqrt(dx * dx + dy * dy);
    double vt = dis/dt;            //Translation speed [m/s]
    double wt = dth/dt;            //Angular velocity [rad/s]
    double vthre = 0.001;          // Lower limit value of vt.
    double wthre = 0.01;           //Lower limit of wt.

    if (vt < vthre)
        vt = vthre;
    if (wt < wthre)
        wt = wthre;

    // When accumulating, calculate the covariance matrix at time t from the covariance matrix sigma at time t-1.
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero(3, 3);
    if (accum)
    {
        Eigen::Matrix3d Jxk;
        calJxk(th, vt, dt, Jxk);
        A = Jxk * cov * Jxk.transpose();
    }

    Eigen::Matrix2d Uk;
    calUk(vt, wt, Uk);

    Eigen::Matrix<double, 3, 2> Juk;
    calJuk(th, dt, Juk);
    Eigen::Matrix3d B = Juk * Uk * Juk.transpose();

    cov = A + B;
}

// Jacobian matrix for robot pose. vt is robot speed, th is robot direction (radians), dt is time
void CovarianceCalculator::calJxk(double th, double vt, double dt,
                                  Eigen::Matrix3d &Jxk)
{
    double cs = cos(th);
    double sn = sin(th);
    Jxk << 1, 0, -vt * dt * sn, 0, 1, vt * dt * cs, 0, 0, 1;
}

void CovarianceCalculator::calUk(double vt, double wt, Eigen::Matrix2d &Uk)
{
    Uk << a1 * vt * vt, 0, 0, a2 * wt * wt;
}

void CovarianceCalculator::calJuk(double th, double dt,
                                  Eigen::Matrix<double, 3, 2> &Juk)
{
    double cs = cos(th);
    double sn = sin(th);

    Juk << dt * cs, 0 , dt*sn, 0, 0, dt;
}

// Decompose only the translational components of the covariance matrix cov into eigenvalues, and put the eigenvalues in vals and the eigenvectors in vec1 and vec2.
double CovarianceCalculator::calEigen(const Eigen::Matrix3d &cov, double *vals,
                                      double *vec1, double *vec2)
{
    //Extract only the translation part
    double cv2[2][2];
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            cv2[i][j] = cov(i, j);
        }         
    }

    MyUtil::calEigen2D(cv2, vals, vec1, vec2); //Eigenvalue decomposition
    double ratio = vals[0]/vals[1];

    //For confirmation
    //printf("Eigen: ratio = %g, val1 = %g, val2 = %g\n", ratio, vals[0], vals[1]);
    //printf("Eigen: vec1=(%g, %g), ang = %g\n", vec1[0], vec1[1],RAD2DEG(atan2(vec1[1], vec1[0])));
    return (ratio);
}

// Accumulation of covariance matrices. 
//Add the covariance matrix mcov of the amount of movement to the covariance matrix prevCov of the previous position to obtain the covariance matrix curCov of the current position.
void CovarianceCalculator::accumulateCovariance(const Pose2D &curPose,
                                                const Pose2D &prevPose,
                                                const Eigen::Matrix3d &prevCov,
                                                const Eigen::Matrix3d &mcov,
                                                Eigen::Matrix3d &curCov)
{
    Eigen::Matrix3d J1, J2;
    J1 << 1, 0, -(curPose.ty - prevPose.ty), 0, 1, curPose.tx - prevPose.tx, 0, 0, 1;

    double prevCos = cos(DEG2RAD(prevPose.th));
    double prevSin = sin(DEG2RAD(prevPose.th));
    J2 << prevCos, -prevSin, 0, prevSin, prevCos, 0, 0, 0, 1;

    curCov = J1 * prevCov * J1.transpose() + J2 * mcov * J2.transpose(); 
}

//Rotate covariance matrix cov by angle of pose
void CovarianceCalculator::rotateCovariance(const Pose2D &pose,
                                            const Eigen::Matrix3d &cov,
                                            Eigen::Matrix3d &icov,
                                            bool reverse)
{
    double th = pose.th;
    double cs = cos(DEG2RAD(th));   // cos due to rotation component th of the pose
    double sn = sin(DEG2RAD(th));
    Eigen::Matrix3d J;   //Jacobian matrix of rotation
    J << cs, -sn, 0, sn, cs, 0, 0, 0, 1;

    Eigen::Matrix3d JT = J.transpose();

    if (reverse)
        icov = JT * cov * J; // inverse rotation
    else
        icov = J * cov * JT; // rotation
}