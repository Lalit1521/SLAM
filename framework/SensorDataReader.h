#ifndef SENSOR_DATA_READER_H_
#define SENSOR_DATA_READER_H_

#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include <fstream>
#include <iostream>
#include <vector>

/////////

class SensorDataReader
{
private:
    int angleOffset;      // Offset between laser scanner and robot orientation
    std::ifstream inFile; // data file

public:
    SensorDataReader() : angleOffset(180) {}

    ~SensorDataReader() {}

    ////////

    bool openScanFile(const char *filepath)
    {
        inFile.open(filepath);
        if (!inFile.is_open())
        {
            std::cerr << "Error: cannot open file " << filepath << std::endl;
            return (false);
        }

        return (true);
    }

    void closeScanFile() { inFile.close(); }

    void setAngleOffset(int o) { angleOffset = o; }

    //////////

    bool loadScan(size_t cnt, Scan2D &scan);
    bool loadLaserScan(size_t cnt, Scan2D &scan);
};

#endif