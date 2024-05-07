#include "SensorDataReader.h"

using namespace std;

// Read one scan from file
bool SensorDataReader::loadScan(size_t cnt, Scan2D &scan)
{
    bool isScan = false;
    while (!inFile.eof() && !isScan)
    { // Continue until reading the scan
        isScan = loadLaserScan(cnt, scan);
    }

    if (isScan)
        return (false); // Meaning there are more files
    else
        return (true); // means the file is finished
}

//////////////

// Read one item from the file. Returns true if the read item is a scan.
bool SensorDataReader::loadLaserScan(size_t cnt, Scan2D &scan)
{
    string type; // Item label in file
    inFile >> type;
    if (type == "LASERSCAN")
    //if (type == "FLASER")
    { // For scanning
        scan.setSid(cnt);

        int sid, sec, nsec;
        inFile >> sid >> sec >> nsec; // don't use these

        vector<LPoint2D> lps;
        int pnum; // Number of scan points
        inFile >> pnum;
        //double start = -90.0; // Start value
        //double end = 90.0; // End value

        // Calculate the step size
        //double step = (end - start)/pnum; // num_readings;
        // Create a vector to store the values
        //std::vector<double> index;

        // Generate the values
        //for (double val = start; val <= end; val += step) 
        //    index.emplace_back(val);

        //for (int i = 0; i < index.size(); i++)
        //{
        //    printf("ANgle = %g\n", index[i]);
        //}
        // Remove the middle element if the number of readings is even
        //if (pnum % 2 == 0)
        //{ 
        //    int middle_index = pnum / 2;
        //    index.erase(index.begin() + middle_index);
        //}

        lps.reserve(pnum);
        for (int i = 0; i < pnum; i++)
        {
            float angle, range;
            inFile >> angle >> range; // Azimuth and distance of scan point
            angle += angleOffset;     // Account for laser scanner directional offset
            //angle = index[i];
            //inFile >> range;
            if (range <= Scan2D::MIN_SCAN_RANGE || range >= Scan2D::MAX_SCAN_RANGE)
            {
                //      if (range <= Scan2D::MIN_SCAN_RANGE || range >= 3.5) { //
                //      Easily cause degeneration on purpose
                continue;
            }

            LPoint2D lp;
            lp.setSid(cnt);         // Set the scan number to cnt (serial number)
            lp.calXY(range, angle); // Calculate the position xy of a point from angle and range
            lps.emplace_back(lp);
        }
        scan.setLps(lps);

        // Odometry information corresponding to the scan
        Pose2D &pose = scan.pose;
        inFile >> pose.tx >> pose.ty;
        double th;
        inFile >> th;
        pose.setAngle(RAD2DEG(th)); // The odometry angle is in radians, so convert it into degrees.
        pose.calRmat();

        return (true);
    }
    else
    { // Other than scanning
        string line;
        getline(inFile, line); // Skip

        return (false);
    }
}