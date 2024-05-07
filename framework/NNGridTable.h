#ifndef _NN_GRID_TABLE_H_
#define _NN_GRID_TABLE_H_

#include "MyUtil.h"
#include "Pose2D.h"
#include <vector>

struct NNGridCell
{
    std::vector<const LPoint2D *> lps;  // Scan points cloud stored in this cell

    void clear()
    {
        lps.clear();   //empty
    }
};

class NNGridTable
{
    public:
        double csize;            //Cell size [m]
        double rsize;            //Size of target area[m]
        int tsize;               //half the table size
        int width;

    public:
        std::vector<NNGridCell> table;  //table body

    public:
        NNGridTable() : csize(0.05), rsize(40)
        {
            tsize = static_cast<int>(rsize / csize);    //half the table size
            size_t w = static_cast<int>(2 * tsize + 1); //table size
            width = w;
            table.resize(w * w);
            clear();                     
        }

        ~NNGridTable() {}

        void clear()
        {
            for (size_t i = 0; i < table.size(); i++)
            {
                table[i].clear();
            }
        }

        void addPoint(const LPoint2D *lp);
        const LPoint2D *findClosestPoint(const LPoint2D *clp, const Pose2D &predPose);
        void makeCellPoints(int nthre, std::vector<LPoint2D> &ps);
        bool occupied(size_t index);
        void indicesToPoint(vector2D &indices, Vector2D &point);
        void pointToIndices(Pose2D &point, vector2D &indices);
};

#endif