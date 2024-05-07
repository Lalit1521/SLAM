#include "SlamBackEnd.h"
#include "P2oDriver2D.h"

using namespace std;

Pose2D SlamBackEnd::adjustPoses()
{
    newPoses.clear();
    
    P2oDriver2D p2o;
    p2o.doP2o(*pg, newPoses, 5); //Repeat 5 times

    return (newPoses.back());
}

void SlamBackEnd::remakeMaps()
{
    //Modification of map
    vector<PoseNode *> &pnodes = pg->nodes;  //Pose Node
    for (size_t i = 0; i < newPoses.size(); i++)
    {
        Pose2D &npose = newPoses[i];
        PoseNode *pnode = pnodes[i];
        pnode->setPose(npose);  
    }

    printf("newPoses.size=%lu, nodes.size=%lu\n", newPoses.size(), pnodes.size());

    pcmap->remakeMaps(newPoses);

}