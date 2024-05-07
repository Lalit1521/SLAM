#include "SlamLauncher.h"

int main(int argc, char *argv[])
{
    bool scanCheck = false;    // Only scan display?
    bool odometryOnly = false; // Map construction using odometry?
    char *filename;            // data file name
    int startN = 0;            // Starting scan number

    if (argc < 2)
    {
        printf("Error: too few arguments.\n");
        return (1);
    }

    // Processing command arguments
    int idx = 1;
    // Interpretation of command options (arguments with '-')
    if (argv[1][0] == '-')
    {
        for (int i = 1;; i++)
        {
            char option = argv[1][i];
            if (option == NULL)
                break;
            else if (option == 's') // Scan display only
                scanCheck = true;
            else if (option == 'o') // Map construction using odometry
                odometryOnly = true;
        }
        if (argc == 2)
        {
            printf("Error: no file name.\n");
            return (1);
        }
        ++idx;
    }
    if (argc >= idx + 1) // '-' idx=2 if present, idx=1 if not
        filename = argv[idx];
    if (argc == idx + 2) // If argc is 2 greater than idx, there is startN
        startN = atoi(argv[idx + 1]);
    else if (argc >= idx + 2)
    {
        printf("Error: invalid arguments.\n");
        return (1);
    }

    printf("SlamLauncher: startN=%d, scanCheck=%d, odometryOnly=%d\n", startN,
           scanCheck, odometryOnly);
    printf("filename=%s\n", filename);

    // open file
    SlamLauncher sl;
    bool flag = sl.setFilename(filename);
    if (!flag)
        return (1);

    sl.setStartN(startN); // Setting the starting scan number

    // Processing body
    if (scanCheck)
        sl.showScans();
    else
    { // Other than scan display, cases are classified within SlamLauncher.
        sl.setOdometryOnly(odometryOnly);
        sl.customizeFramework();
        sl.run();
        //printf("good\n");
    }

    return (0);
}