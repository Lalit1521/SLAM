#include "MapDrawer.h"

using namespace std;

////////// Gnuplot //////////

// Draw a map and trajectory
void MapDrawer::drawMapGp(const PointCloudMap &pcmap)
{
    const vector<LPoint2D> &lps = pcmap.globalMap; // map point cloud
    const vector<Pose2D> &poses = pcmap.poses;     // robot trajectory
    const vector<Vector2D> &pathPoses = pcmap.plan;
    drawGp(lps, poses, pathPoses);
}

// Draw 1 scan
void MapDrawer::drawScanGp(const Scan2D &scan)
{
    vector<Pose2D> poses;
    Pose2D pose;              // origin
    poses.emplace_back(pose); // Put it in a vector to use drawGp
    //drawGp(scan.lps, poses);
}

// Draw only the robot trajectory
void MapDrawer::drawTrajectoryGp(const vector<Pose2D> &poses)
{
    vector<LPoint2D> lps; // Dummy (empty) for using drawGp
    //drawGp(lps, poses);
}

//////////

void MapDrawer::drawGp(const vector<LPoint2D> &lps, const vector<Pose2D> &poses, const vector<Vector2D> &pathPoses,
                       bool flush)
{
    printf("drawGp: lps.size=%lu\n", lps.size()); // For checking the score

    // gnuplot settings
    fprintf(gp, "set multiplot\n");
    //  fprintf(gp, "plot '-' w p pt 7 ps 0.1, '-' with vector\n");
    //fprintf(gp, "plot '-' w p pt 7 ps 0.1 lc rgb 0x0, '-' with vector\n");
    fprintf(gp, "plot '-' w p pt 7 ps 0.1 lc rgb 0x0, '-' with vector, '-' with lines lc rgb 'blue', '-' w p pt 7 ps 1.0 lc 'green 0x0,\n");
    //fprintf(gp, "plot '-' w p pt 7 ps 0.1 lc rgb 0x0, '-' with vector, '-' with lines lc rgb 'blue',\n");

    // Point cloud drawing
    int step1 = 1; // Point thinning interval. Make it bigger when drawing is heavy
    for (size_t i = 0; i < lps.size(); i += step1)
    {
        const LPoint2D &lp = lps[i];
        fprintf(gp, "%lf %lf\n", lp.x, lp.y); // drawing points
    }
    fprintf(gp, "e\n");

    /*
    // Drawing robot trajectory
    int step2 = 10; // Robot position thinning interval
    for (size_t i = 0; i < poses.size(); i += step2)
    {
        const Pose2D &pose = poses[i];
        double cx = pose.tx; // translational position
        double cy = pose.ty;
        double cs = pose.Rmat[0][0]; // cos due to rotation angle
        double sn = pose.Rmat[1][0]; // sin due to rotation angle

        // Draw the position and orientation of the robot coordinate system
        double dd = 0.4;
        double x1 = cs * dd; // x-axis of robot coordinate system
        double y1 = sn * dd;
        double x2 = -sn * dd; // y-axis of robot coordinate system
        double y2 = cs * dd;
        fprintf(gp, "%lf %lf %lf %lf\n", cx, cy, x1, y1);
        fprintf(gp, "%lf %lf %lf %lf\n", cx, cy, x2, y2);
    }
    fprintf(gp, "e\n");
    */

    // Drawing robot trajectory
    const Pose2D &pose = poses.back();
    double cx = pose.tx; // translational position
    double cy = pose.ty;
    double cs = pose.Rmat[0][0]; // cos due to rotation angle
    double sn = pose.Rmat[1][0]; // sin due to rotation angle

    // Draw the position and orientation of the robot coordinate system
    double dd = 0.4;
    double x1 = cs * dd; // x-axis of robot coordinate system
    double y1 = sn * dd;
    double x2 = -sn * dd; // y-axis of robot coordinate system
    double y2 = cs * dd;
    fprintf(gp, "%lf %lf %lf %lf\n", cx, cy, x1, y1);
    fprintf(gp, "%lf %lf %lf %lf\n", cx, cy, x2, y2);
    fprintf(gp, "e\n");
    

    for (size_t i = 0; i < pathPoses.size(); i += 1)
    {
        const Vector2D pos = pathPoses[i];
        //printf("Points : %lf %lf\n", pos.x, pos.y);
        fprintf(gp, "%lf %lf\n", pos.x, pos.y);
    }
    fprintf(gp, "e\n");

    fprintf(gp, "%lf %lf\n", -2.0, -2.0);
    fprintf(gp, "e\n");

    if (flush)
        fflush(gp); // Write out buffer data. If you don't do this, the drawing won't look good.
}