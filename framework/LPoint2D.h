#ifndef LPOINT2D_H_
#define LPOINT2D_H_

#include "MyUtil.h"

struct Vector2D
{
    double x;
    double y;

    Vector2D(double dx, double dy) : x(dx), y(dy) {} 

    Vector2D() {}
};

struct vector2D
{
    int x, y;

    vector2D() : x(0), y(0) {}

    vector2D(int dx, int dy) : x(dx), y(dy) {}

    bool operator==(const vector2D& other) const 
    {
        return (x == other.x && y == other.y);
    }

    bool operator<(const vector2D& other) const {
        // Compare based on x-coordinate first
        if (x < other.x) return true;
        if (x > other.x) return false;
        // If x-coordinates are equal, compare based on y-coordinate
        return y < other.y;
    }
};

enum ptype
{
    UNKNOWN = 0,
    LINE = 1,
    CORNER = 2,
    ISOLATE = 3
}; // Point type: unknown, straight, corner, isolated

struct LPoint2D
{
    int sid;      //Frame number (scan number)
    double x;     //position x
    double y;     //position y
    double nx;    //normal vector
    double ny;    //normal vector
    double atd;   //accumlated travel distance
    ptype type;   //point type

    LPoint2D(): sid(0), x(0), y(0) {init();}

    LPoint2D(int id, double _x, double _y) : x(_x), y(_y)
    {
        init();
        sid = id;
    }

    void init()
    {
        sid = -1;
        atd = 0;
        type = UNKNOWN;
        nx = 0;
        ny = 0;
    }

    void setData(int id, double _x, double _y)
    {
        init();
        sid = id;
        x = _x;
        y = _y;
    }

    void setXY(double _x, double _y)
    {
        x = _x;
        y = _y;
    }

    // Find xy from range and angle (right-handed)
    void calXY(double range, double angle)
    {
        double a = DEG2RAD(angle);
        x = range * cos(a);
        y = range * sin(a);
    }

    
    // Find xy from range and angle (left handed system)
    void calXYi(double range, double angle)
    {
        double a = DEG2RAD(angle);
        x = range * cos(a);
        y = -range * sin(a);
    }

    void setSid(int i) 
    {
        sid = i;
    }

    void setAtd(double t) { atd = t;}

    void setType(ptype t) { type = t;}

    void setNormal(double x, double y)
    {
        nx = x;
        ny = y;
    }

};

#endif