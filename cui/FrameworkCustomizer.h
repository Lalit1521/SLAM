#ifndef FRAMEWORK_CUSTOMIZER_H_
#define FRAMEWORK_CUSTOMIZER_H_

#include "CostFunction.h"
#include "CostFunctionED.h"
#include "CostFunctionPD.h"
#include "DataAssociator.h"
#include "DataAssociatorLS.h"
#include "DataAssociatorGT.h"
#include "MyUtil.h"
#include "PointCloudMap.h"
#include "PointCloudMapBS.h"
#include "PointCloudMapGT.h"
#include "PointCloudMapLP.h"
#include "PoseEstimatorICP.h"
#include "PoseOptimizer.h"
#include "PoseOptimizerSD.h"
#include "PoseOptimizerSL.h"
#include "PoseFuser.h"
#include "RefScanMaker.h"
#include "RefScanMakerBS.h"
#include "RefScanMakerLM.h"
#include "ScanMatcher2D.h"
#include "ScanPointResampler.h"
#include "ScanPointAnalyser.h"
#include "SlamFrontEnd.h"
#include "LoopDetector.h"
#include "LoopDetectorSS.h"
#include "PathPlanning.h"
#include <vector>

class FrameworkCustomizer
{
    // Parts for framework modification
    RefScanMakerBS rsmBS;
    RefScanMakerLM rsmLM;
    DataAssociatorLS dassLS;
    DataAssociatorGT dassGT;
    CostFunctionED cfuncED;
    CostFunctionPD cfuncPD;
    PoseOptimizerSD poptSD;
    PoseOptimizerSL poptSL;
    PointCloudMapBS pcmapBS;
    PointCloudMapGT pcmapGT;
    PointCloudMapLP pcmapLP;
    PointCloudMap *pcmap; // Make it a member variable to reference in SlamLauncher
    ScanPointResampler spres;
    ScanPointAnalyser spana;
    LoopDetector lpdDM;
    LoopDetectorSS lpdSS;
    
    PoseEstimatorICP poest;
    PoseFuser pfu;
    ScanMatcher2D smat;
    SlamFrontEnd *sfront;
    PathPlanning path;

public:
    FrameworkCustomizer() : pcmap(nullptr) {}

    ~FrameworkCustomizer() {}

    void setSlamFrontEnd(SlamFrontEnd *f) { sfront = f; }

    PointCloudMap *getPointCloudMap() { return (pcmap); }

    //////

    void makeFramework();
    void customizeA();
    void customizeB();
    void customizeC();
    void customizeD();
    void customizeE();
    void customizeF();
    void customizeG();
    void customizeH();
    void customizeI();

};

#endif