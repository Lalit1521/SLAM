#include "MotionPlanning.h"

using namespace std;

double MotionPlanning::LOOKAHEAD_DISTANCE = 0.5;
double MotionPlanning::KP = 1.0;

void MotionPlanning::command()
{
    Pose2D pose = pcmap->poses.back();
    double x = pose.tx;
    double y = pose.ty;
    double theta = pose.th;
    int lookAheadIndex = -1;

    vector<Vector2D> &path = pcmap->plan;
    for (int i = 0; i < path.size(); i++)
    {
        if (sqrt(pow((path[i].x - x),2) + pow((path[i].y - y),2)) > LOOKAHEAD_DISTANCE)
        {
            lookAheadIndex = i;
            break;
        }
    }

    if (lookAheadIndex == -1)
    {
        double real_distance = sqrt((path.back().x - x) * (path.back().x - x) + (path.back().y - y) * (path.back().y - y));
        double lookahead_angle = atan2(path[lookAheadIndex].y - y, path[lookAheadIndex].x - x);
        double del_y = real_distance * sin(lookahead_angle - theta);
        steering_angle = KP * 2.00 * del_y / (real_distance * real_distance);
        speed = 0.5 * real_distance;
    }
    else
    {
        double real_distance = sqrt((path[lookAheadIndex].x - x) * (path[lookAheadIndex].x - x) + (path[lookAheadIndex].y - y)* (path[lookAheadIndex].y - y));
        double lookahead_angle = atan2(path[lookAheadIndex].y - y, path[lookAheadIndex].x - x);
        double del_y = real_distance * sin(lookahead_angle - theta);
        steering_angle = KP * 2.00 * del_y / (real_distance * real_distance);
        speed = 0.5;
    }
    
    

}