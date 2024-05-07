#include "PoseGraph.h"

using namespace std;

//Add node to pose graph
PoseNode *PoseGraph::addNode(const Pose2D &pose)
{
    PoseNode *n1 = allocNode();  //Node generation
    addNode(n1, pose);           //Add node to pose graph
    return (n1);
}

//Add node to pose graph
void PoseGraph::addNode(PoseNode *n1, const Pose2D &pose)
{
    n1->setNid((int)nodes.size()); // Assign node ID. Same as node serial number
    n1->setPose(pose);             //Set robot position
    nodes.push_back(n1);           //Add to the end of nodes
}

//Find node entity from node ID(nid)
PoseNode *PoseGraph::findNode(int nid)
{
    for (size_t i = 0; i < nodes.size(); i++)
    {//Simply linear search
        PoseNode *n = nodes[i];
        if (n->nid == nid)
            return (n);
    }

    return (nullptr);
}

//Add an arc to the pose graph
void PoseGraph::addArc(PoseArc *arc)
{
    arc->src->addArc(arc);  //Add arc to start node
    arc->dst->addArc(arc);  //Add arc to end node
    arcs.push_back(arc);    //Add arc arc to end of arcs
}

// Generate an arc between the start node srcNid and the end node dstNid
PoseArc *PoseGraph::makeArc(int srcNid, int dstNid, const Pose2D &relPose,
                            const Eigen::Matrix3d &cov)
{   //Inverse of covariance matrix is information matrix
    Eigen::Matrix3d inf = MyUtil::svdInverse(cov);

    PoseNode *src = nodes[srcNid];  //Starting node
    PoseNode *dst = nodes[dstNid];  //end node

    PoseArc *arc = allocArc();  //Arc generation
    arc->setup(src, dst, relPose, inf);  //relPose is the relative position measured

    return (arc);  
}

//Return an anc whose start node is srcNid and end node is dstNid
PoseArc *PoseGraph::findArc(int srcNid, int dstNid)
{
    for (size_t i = 0; i < arcs.size(); i++)
    {
        PoseArc *a = arcs[i];
        if (a->src->nid == srcNid && a->dst->nid == dstNid)
            return (a);
    }
    return (nullptr);
}

// For confirmation
void PoseGraph::printNodes()
{
    printf("--- printNodes ---\n");
    printf("nodes.size=%lu\n", nodes.size());
    for (size_t i = 0; i < nodes.size(); i++)
    {
        PoseNode *node = nodes[i];
        printf("i=%lu: nid=%d, tx=%g, ty=%g, th=%g\n", i, node->nid, node->pose.tx,
               node->pose.ty, node->pose.th);

        for (size_t j = 0; j < node->arcs.size(); j++)
        {
            PoseArc *a = node->arcs[j];
            printf("arc j=%lu: srcId=%d, dstId=%d, src=%p, dst=%p\n", j, a->src->nid,
                   a->dst->nid, a->src, a->dst);
        }
    }
}

// For confirmation
void PoseGraph::printArcs()
{
    printf("--- printArcs ---\n");
    printf("arcs.size=%lu\n", arcs.size());
    for (size_t j = 0; j < arcs.size(); j++)
    {
        PoseArc *a = arcs[j];
        double dis = (a->src->pose.tx - a->dst->pose.tx) *
                         (a->src->pose.tx - a->dst->pose.tx) +
                     (a->src->pose.ty - a->dst->pose.ty) *
                         (a->src->pose.ty - a->dst->pose.ty);

        Pose2D &rpose = a->relPose;
        printf("j=%lu, srcId=%d, dstId=%d, tx=%g, ty=%g, th=%g\n", j, a->src->nid,
               a->dst->nid, rpose.tx, rpose.ty, rpose.th);
    }
}