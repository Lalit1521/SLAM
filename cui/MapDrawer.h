#include "LPoint2D.h"
#include "MyUtil.h"
#include "PointCloudMap.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include <stdio.h>
#include <vector>

class MapDrawer
{
private:
    FILE *gp;    // pipe to gnuplot
    double xmin; // Drawing range [m]
    double xmax;
    double ymin;
    double ymax;
    double aspectR; // xy ratio

public:
    MapDrawer()
        : gp(nullptr), xmin(-10), xmax(10), ymin(-10), ymax(10), aspectR(-1.0) {}

    ~MapDrawer() { finishGnuplot(); }

    /////////

    void initGnuplot()
    {
#ifdef _WIN32
        gp = _popen("gnuplot", "w"); // Pipe open. Windows
#elif __linux__
        gp = popen("gnuplot", "w"); // Pipe open.Linux
#endif
    }

    void finishGnuplot()
    {
        if (gp != nullptr)
#ifdef _WIN32
            _pclose(gp);
#elif __linux__
            pclose(gp);
#endif
    }

    void setAspectRatio(double a)
    {
        aspectR = a;
        fprintf(gp, "set size ratio %lf\n", aspectR);
    }

    void setRange(double R)
    { // Make the drawing area R square
        xmin = ymin = -R;
        xmax = ymax = R;
        fprintf(gp, "set xrange [%lf:%lf]\n", xmin, xmax);
        fprintf(gp, "set yrange [%lf:%lf]\n", ymin, ymax);
    }

    void setRange(double xR, double yR)
    { // Set the drawing range to ±xR, ±yR
        xmin = -xR;
        xmax = xR;
        ymin = -yR;
        ymax = yR;
        fprintf(gp, "set xrange [%lf:%lf]\n", xmin, xmax);
        fprintf(gp, "set yrange [%lf:%lf]\n", ymin, ymax);
    }

    void setRange(double xm, double xM, double ym,
                  double yM)
    { // Specify the entire drawing range
        xmin = xm;
        xmax = xM;
        ymin = ym;
        ymax = yM;
        fprintf(gp, "set xrange [%lf:%lf]\n", xmin, xmax);
        fprintf(gp, "set yrange [%lf:%lf]\n", ymin, ymax);
    }

    ////////

    void drawMapGp(const PointCloudMap &pcmap);
    void drawScanGp(const Scan2D &scan);
    void drawTrajectoryGp(const std::vector<Pose2D> &poses);
    void drawGp(const std::vector<LPoint2D> &lps,
                const std::vector<Pose2D> &poses, const std::vector<Vector2D > &pathPoses, bool flush = true);
};