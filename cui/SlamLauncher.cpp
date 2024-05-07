#include "SlamLauncher.h"
//#include "ScanPointResampler.h"
#include <boost/algorithm/string/predicate.hpp>
#include <boost/timer.hpp>

using namespace std; // Use C++ Standard Library namespaces

//////////

void SlamLauncher::run()
{
    mdrawer.initGnuplot(); // gnuplot initialization
    mdrawer.setAspectRatio(
        -0.9); // Ratio of x-axis to y-axis (if negative, content is constant)

    size_t cnt = 0; // Logical time of processing
    if (startN > 0)
        skipData(startN); // skip data to startN

    double totalTime = 0, totalTimeDraw = 0, totalTimeRead = 0;
    Scan2D scan;
    bool eof = sreader.loadScan(cnt, scan); // Read one scan from file
    boost::timer tim;
    while (!eof)
    {
        if (odometryOnly)
        { // Map construction by odometry (preferred over SLAM)
            if (cnt == 0)
            {
                ipose = scan.pose;
                ipose.calRmat();
            }
            mapByOdometry(&scan);
        }
        else
            sfront.process(scan); // Map construction by SLAM

        double t1 = 1000 * tim.elapsed();

        if (cnt % drawSkip == 0)
        { // drawSkip
            mdrawer.drawMapGp(*pcmap);
        }
        double t2 = 1000 * tim.elapsed();

        ++cnt;                             // Logical time update
        eof = sreader.loadScan(cnt, scan); // Load next scan

        double t3 = 1000 * tim.elapsed();
        totalTime = t3;             // Total Processing Time
        totalTimeDraw += (t2 - t1); // Total drawing time
        totalTimeRead += (t3 - t2); // Total load time

        printf("---- SlamLauncher: cnt=%lu ends ----\n", cnt);
    }
    sreader.closeScanFile();

    printf("Elapsed time: mapping=%g, drawing=%g, reading=%g\n",
           (totalTime - totalTimeDraw - totalTimeRead), totalTimeDraw,
           totalTimeRead);
    printf("SlamLauncher finished.\n");

    // To keep the drawing screen after the process is finished, use sleep to make
    // an infinite loop. ctrl-C to exit.
    while (true)
    {
#ifdef _WIN32
        Sleep(1000); // Sleep on Windows
#elif __linux__
        usleep(1000000); // Sleep on Linux
#endif
    }
}

// Skip reading from start to num scans
void SlamLauncher::skipData(int num)
{
    Scan2D scan;
    bool eof = sreader.loadScan(0, scan);
    for (int i = 0; !eof && i < num; i++)
    { // Read num blanks
        eof = sreader.loadScan(0, scan);
    }
}

///////// Map construction by odometry //////////

void SlamLauncher::mapByOdometry(Scan2D *scan)
{
    //  Pose2D &pose = scan->pose;               // Odometry position at the time
    //  of scan acquisition
    Pose2D pose;
    Pose2D::calRelativePose(scan->pose, ipose, pose);
    vector<LPoint2D> &lps = scan->lps; // scan point cloud
    std::vector<LPoint2D> glps;        // Point cloud in map coordinate system
    for (size_t j = 0; j < lps.size(); j++)
    {
        LPoint2D &lp = lps[j];
        LPoint2D glp;
        pose.globalPoint(
            lp,
            glp); // Convert from sensor coordinate system to map coordinate system
        glps.emplace_back(glp);
    }

    // Store data in point cloud map pcmap
    pcmap->addPose(pose);
    pcmap->addPoints(glps);
    pcmap->makeGlobalMap();

    printf("Odom pose: tx=%g, ty=%g, th=%g\n", pose.tx, pose.ty, pose.th);
}

////////// scan drawing ////////

void SlamLauncher::showScans()
{
    mdrawer.initGnuplot();
    mdrawer.setRange(6); // Drawing range. If the scan is 6m square
    mdrawer.setAspectRatio(
        -0.9); // Ratio of x-axis to y-axis (if negative, content is constant)

    size_t cnt = 0;
    if (startN > 0)
        skipData(startN); 

    Scan2D scan;
    bool eof = sreader.loadScan(cnt, scan);
    while (!eof)
    {
        //    spres.resamplePoints(&scan);         

#ifdef _WIN32
        Sleep(100); // Windows on Sleep
#elif __linux__
        usleep(100000);  // Linux on sleep
#endif

        mdrawer.drawScanGp(scan); 

        printf("---- scan num=%lu ----\n", cnt);
        eof = sreader.loadScan(cnt, scan);
        ++cnt;
    }
    sreader.closeScanFile();
    printf("SlamLauncher finished.\n");
}


bool SlamLauncher::setFilename(char *filename)
{
    bool flag = sreader.openScanFile(filename); 

    return (flag);
}

////////////

void SlamLauncher::customizeFramework()
{
    fcustom.setSlamFrontEnd(&sfront);
    fcustom.makeFramework();
    fcustom.customizeI(); 

    pcmap = fcustom.getPointCloudMap(); 
}