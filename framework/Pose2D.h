#ifndef _POSE2D_H_
#define _POSE2D_H_

#include "LPoint2D.h"
#include "MyUtil.h"

struct Pose2D
{
    double tx;           // translation x
    double ty;           // translation y
    double th;           // rotation angle
    double Rmat[2][2];   // Pose rotation matrix

    Pose2D() : tx(0), ty(0), th(0)
    {
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j <2; j++)
            {
                Rmat[i][j] = (i == j)? 1.0 : 0.0;
            }
        }
    }

    Pose2D(double tx, double ty, double th)
    {
        this->tx = tx;
        this->ty = ty;
        this->th = th;
        calRmat();
    }

    Pose2D(double mat[2][2], double tx, double ty, double th)
    {
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                Rmat[i][j] = mat[i][j];
            }
        }
        this->tx = tx;
        this->ty = ty;
        this->th = th;
    }

    void reset()
    {
        tx = ty = th = 0;
        calRmat();
    }

    void setVal(double x, double y, double a)
    {
        tx = x;
        ty = y;
        th = a;
        calRmat();
    }

    void calRmat()
    {
        double a = DEG2RAD(th);
        Rmat[0][0] = Rmat[1][1] = cos(a);
        Rmat[1][0] = sin(a);
        Rmat[0][1] = -Rmat[1][0];
    }

    void setTranslation(double tx, double ty)
    {
        this->tx = tx;
        this->ty = ty;
    }

    void setAngle(double th)
    {
        this->th = th;
    }

    LPoint2D relativePoint(const LPoint2D &p) const;
    LPoint2D globalPoint(const LPoint2D &p) const;
    void globalPoint(const LPoint2D &pi, LPoint2D &po) const;

    static void calRelativePose(const Pose2D &npose, const Pose2D &bpose,
                                Pose2D &relpose);
    static void calGlobalPose(const Pose2D &relPose, const Pose2D &bpose,
                              Pose2D &npose);

};

struct PoseCov
{
    Pose2D pose;
    Eigen::Matrix3d cov;

    PoseCov() {}

    PoseCov(Pose2D &p, Eigen::Matrix3d &c)
    {
        pose = p;
        cov = c;
    }
};
#endif