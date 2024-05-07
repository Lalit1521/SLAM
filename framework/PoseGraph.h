#ifndef POSE_GRAPH_H_
#define POSE_GRAPH_H_

#include "MyUtil.h"
#include "Pose2D.h"
#include <vector>

struct PoseArc;
// vertices of pose graph
struct PoseNode
{
    int nid;           //Node ID: PoseGrpah nodes index
    Pose2D pose;       //Robot position of this node
    std::vector<PoseArc *> arcs; //Arc leading to this node

    PoseNode() : nid(-1) {}

    PoseNode(const Pose2D &p) : nid(-1) { pose = p; }

    ~PoseNode() {}

    void init()
    {
        nid = -1;
        arcs.clear();
    }

    void setPose(const Pose2D &p) { pose = p; }

    void setNid(int n) { nid = n; }

    void addArc(PoseArc *a) { arcs.push_back(a); }
};


//Pose graph edge
struct PoseArc
{
    PoseNode *src;     //the node on the start side of this arc
    PoseNode *dst;     //the end of this arc
    Pose2D relPose;    //Relative position (measured value) of this arc
    Eigen::Matrix3d inf; //information matrix

    PoseArc(void) : src(nullptr), dst(nullptr) {}

    PoseArc(PoseNode *s, PoseNode *d, Pose2D &rel, const Eigen::Matrix3d _inf)
    {
        setup(s, d, rel, _inf);
    }

    ~PoseArc(void) {}

    void setup(PoseNode *s, PoseNode *d, const Pose2D &rel,
                const Eigen::Matrix3d _inf)
    {
        src = s;
        dst = d;
        relPose = rel;
        inf = _inf;
    }
};

class PoseGraph
{
private:
    static const int POOL_SIZE = 100000;
    std::vector<PoseNode> nodePool;  //Memory pool for node generation
    std::vector<PoseArc> arcPool;    //Memory pool for arc generation

public:
    std::vector<PoseNode *> nodes;  //collection of nodes
    std::vector<PoseArc *> arcs; //A collection of arcs.Arc has only one direction

    PoseGraph()
    {
        nodePool.reserve(POOL_SIZE);
        arcPool.reserve(POOL_SIZE);
    }

    ~PoseGraph() {}

    void reset()
    {
        nodes.clear();
        arcs.clear();
        nodePool.clear();
        arcPool.clear();
    }

    //Generating nodes
    PoseNode *allocNode()
    {
        if (nodePool.size() >= POOL_SIZE)
        {
            printf("Error: exceed nodePool capacity\n");
            return (nullptr);
        }

        PoseNode node;
        nodePool.emplace_back(node);
        return (&(nodePool.back()));
    }

    //Arc generation
    PoseArc *allocArc()
    {
        if (arcPool.size() >= POOL_SIZE)
        {
            printf("Error: exceeds arcPool capacity\n");
            return (nullptr);
        }

        PoseArc arc;
        arcPool.emplace_back(arc); //Add it to the memory pool and reference it
        return (&(arcPool.back()));
    }

    PoseNode *addNode(const Pose2D &pose);
    void addNode(PoseNode *n1, const Pose2D &pose);
    PoseNode *findNode(int nid);
    
    void addArc(PoseArc *arc);
    PoseArc *makeArc(int srcNid, int dstNid, const Pose2D &relPose,
                     const Eigen::Matrix3d &cov);
    PoseArc *findArc(int srcNid, int dstNid);

    void printNodes();
    void printArcs();
};

#endif
