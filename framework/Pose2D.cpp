#include "Pose2D.h"

// Convert point p in the global coordinate system to your (Pose2D) local coordinate
LPoint2D Pose2D::relativePoint(const LPoint2D &p) const
{
    double dx = p.x - tx;
    double dy = p.y - ty;
    double x = dx * Rmat[0][0] + dy * Rmat[1][0]; //rotation inverse matrix
    double y = dx * Rmat[0][1] + dy * Rmat[1][1];
    return LPoint2D(p.sid, x, y);
}

// Convert point p in your (Pose2D) local coordinate system to the global coordinate system
LPoint2D Pose2D::globalPoint(const LPoint2D &p) const
{
    double x = Rmat[0][0] * p.x + Rmat[0][1] * p.y + tx;
    double y = Rmat[1][0] * p.x + Rmat[1][1] * p.y + ty;
    return LPoint2D(p.sid, x, y);
}

// Convert the point p in your (Pose2D) local coordinate system to the global coordinate system and put it in po
void Pose2D::globalPoint(const LPoint2D &pi, LPoint2D &po) const
{
    po.x = Rmat[0][0] * pi.x + Rmat[0][1] * pi.y + tx;
    po.y = Rmat[1][0] * pi.x + Rmat[1][1] * pi.y + ty;
}

// Find the relative position relPose of the current coordinate system npose as
// seen from the reference coordinate system bpose（Inverse compounding
// operator）
void Pose2D::calRelativePose(const Pose2D &npose, const Pose2D &bpose,
                             Pose2D &relPose)
{
    const double(*R0)[2] = bpose.Rmat; // Reference coordinate system
    const double(*R1)[2] = npose.Rmat; // Current coordinate system
    double(*R2)[2] = relPose.Rmat;     // relative position

    // translation
    double dx = npose.tx - bpose.tx;
    double dy = npose.ty - bpose.ty;
    relPose.tx = R0[0][0] * dx + R0[1][0] * dy;
    relPose.ty = R0[0][1] * dx + R0[1][1] * dy;

    // rotate
    double th = npose.th - bpose.th;
    if (th < -180)
        th += 360;
    else if (th >= 180)
        th -= 360;
    relPose.th = th;

    relPose.calRmat();
}

// Find the coordinate system npose, which is advanced by the relative position
// relPose from the reference coordinate system bpose（Compounding operator）
void Pose2D::calGlobalPose(const Pose2D &relPose, const Pose2D &bpose,
                           Pose2D &npose)
{
    const double(*R0)[2] = bpose.Rmat;   // Reference coordinate system
    const double(*R1)[2] = relPose.Rmat; // relative position
    double(*R2)[2] = npose.Rmat;         // New coordinate system

    // translation
    double tx = relPose.tx;
    double ty = relPose.ty;
    npose.tx = R0[0][0] * tx + R0[0][1] * ty + bpose.tx;
    npose.ty = R0[1][0] * tx + R0[1][1] * ty + bpose.ty;

    // angle
    double th = bpose.th + relPose.th;
    if (th < -180)
        th += 360;
    else if (th >= 180)
        th -= 360;
    npose.th = th;

    npose.calRmat();
}