#ifndef SLAM_LAUNCHER_H_
#define SLAM_LAUNCHER_H_

#include <vector>
#ifdef _WIN32
#include <windows.h>
#elif __linux__
#include <unistd.h>
#endif

#include "FrameworkCustomizer.h"
#include "MapDrawer.h"
#include "PointCloudMap.h"
#include "SensorDataReader.h"
//#include "SlamBackEnd.h"
#include "SlamFrontEnd.h"

/////////////

class SlamLauncher
{
public:
    int startN;        // Starting scan number
    int drawSkip;      // drawing interval
    bool odometryOnly; // Map construction using odometry?
    Pose2D ipose;      // Auxiliary data for odometry map construction. Set the initial position angle to 0

    Pose2D lidarOffset; // Relative position of laser scanner and robot

    SensorDataReader sreader;    // Reading sensor data from file
    PointCloudMap *pcmap;        // point cloud map
    SlamFrontEnd sfront;         // SLAM front end
    MapDrawer mdrawer;           // Drawing with gnuplot
    FrameworkCustomizer fcustom; // Framework modification

public:
    SlamLauncher()
        : startN(0), drawSkip(1), odometryOnly(false), pcmap(nullptr) {}

    ~SlamLauncher() {}

    ///////////

    void setStartN(int n) { startN = n; }

    void setOdometryOnly(bool p) { odometryOnly = p; }

    ///////////

    void run();
    void showScans();
    void mapByOdometry(Scan2D *scan);
    bool setFilename(char *filename);
    void skipData(int num);
    void customizeFramework();
};

#endif