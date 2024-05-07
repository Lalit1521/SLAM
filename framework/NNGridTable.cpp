#include "NNGridTable.h"

using namespace std;

//Register scan points in grid table
void NNGridTable::addPoint(const LPoint2D *lp)
{
    //Index Calculation for table search.Check whether it is within the target area
    int xi = static_cast<int>(lp->x / csize) + tsize;
    if (xi < 0 || xi > 2* tsize)  //outside the target area
        return;
    int yi = static_cast<int>(lp->y / csize) + tsize;
    if (yi < 0 || yi > 2 * tsize) //outside the target area
        return;

    size_t idx =
        static_cast<size_t>(yi * ( 2* tsize + 1) + xi);  //table index
    table[idx].lps.push_back(lp);          
}

//Create a representative point for each cell in the grid table and store it in ps
void NNGridTable::makeCellPoints(int nthre, vector<LPoint2D> &ps)
{
    //Currently, the scan numbers of each point within the cell are averaged.
    size_t nn = 0;
    for (size_t i = 0; i < table.size(); i++)
    {
        vector<const LPoint2D *> &lps = table[i].lps;  // Cell scan point cloud
        nn += lps.size();
        if (lps.size() >= nthre)
        {                           //Process only cells greater than nthre
            double gx = 0, gy = 0;  //Center of gravity of point cloud
            double nx = 0, ny = 0;  //Average normal vector of point cloud
            int sid = 0;
            for (size_t j = 0; j <lps.size(); j++)
            {
                const LPoint2D *lp = lps[j];
                gx += lp->x;  //Accumulate points
                gy += lp->y;
                nx += lp->nx; //Accumulate normal vectors
                ny += lp->ny;
                sid += lp->sid;  //When taking average number of scans
            }
            gx /= lps.size();
            gy /= lps.size();
            double L = sqrt(nx * nx + ny * ny);
            nx /= L;   
            ny /= L;
            sid /= lps.size();

            LPoint2D newLp(sid, gx, gy);  //Generate cell representative points
            newLp.setNormal(nx, ny);      //Normal vector settings
            newLp.setType(LINE);
            ps.emplace_back(newLp);       //Add to global map which is variable ps
        }

    }
}


// Find the closest point from the grid table to the position where scan point clp is coordinate-transformed using predPose
const LPoint2D *NNGridTable::findClosestPoint(const LPoint2D *clp, 
                                              const Pose2D &predPose)
{
    LPoint2D glp;                         //predicted position of crp
    predPose.globalPoint(*clp, glp);      //Coordinate transformation with relPose

    //clp table index.Check if it is within the target area.
    int cxi = static_cast<int>(glp.x / csize) + tsize;
    if (cxi < 0 || cxi > 2 * tsize)
        return (nullptr);
    int cyi = static_cast<int>(glp.y / csize) + tsize;
    if (cyi < 0 || cyi > 2* tsize)
        return (nullptr);
        
    size_t pn = 0;  //Total number of cell in the searched cells.For debugging
    double dmin = 1000000;
    const LPoint2D *lpmin = nullptr;  //closest point(destination point)
    double dthre = 0.2;
    int R = static_cast<int>(dthre / csize);

    //Search for +-R squares
    for (int i = -R; i <= R; i++)
    {
        int yi = cyi + i; //expand from cyi
        if (yi < 0 || yi > 2 * tsize)
            continue;
        for (int j = -R; j <= R; j++)
        {
            int xi = cxi + j;  //expand from cxi
            if (xi < 0 || xi > 2 * tsize)
                continue;
            
            size_t idx = yi * (2 * tsize + 1) + xi;  //table index
            NNGridCell &cell = table[idx];
            vector<const LPoint2D *> &lps = cell.lps; 
            for (size_t k = 0; k < lps.size(); k++)
            {
                const LPoint2D *lp = lps[k];
                double d = (lp->x - glp.x) * (lp->x - glp.x) +
                            (lp->y - glp.y) * (lp->y - glp.y);

                if (d <= dthre * dthre && d < dmin)
                {
                    dmin = d;
                    lpmin = lp;
                }
            }

        }
    }
    return (lpmin);

}

bool NNGridTable::occupied(size_t index)
{   
    //Check whether the index is occupied.
    vector<const LPoint2D *> &lps = table[index].lps;
    if (lps.size() > 0)
        return true;
    return false;
    
} 

void NNGridTable::indicesToPoint(vector2D &indices, Vector2D &point)
{
    //Convert from indices to point.
    point.x = (indices.x - tsize) * csize; 
    point.y = (indices.y - tsize) * csize;
}

void NNGridTable::pointToIndices(Pose2D &point, vector2D &indices)
{
    //Convert from point to indices.
    indices.x = static_cast<int>(point.tx / csize) + tsize;
    indices.y = static_cast<int>(point.ty / csize) + tsize;
}