#include "FrameworkCustomizer.h"

using namespace std;

// Set up the basics of the framework
void FrameworkCustomizer::makeFramework()
{
    smat.setPoseEstimator(&poest);
    smat.setPoseFuser(&pfu);

    lpdSS.setPoseEstimator(&poest);
    lpdSS.setPoseFuser(&pfu);
    lpdSS.setDataAssociator(&dassGT);
    lpdSS.setCostFunction(&cfuncPD);
    lpdSS.setPointCloudMap(&pcmapLP);

    sfront->setScanMatcher(&smat);
}

// Framework basic configuration
void FrameworkCustomizer::customizeA()
{
    pcmap = &pcmapBS;               // Point cloud map that stores all scan points
    RefScanMaker *rsm = &rsmBS;     // Set the previous scan as the reference scan
    DataAssociator *dass = &dassLS; // Data matching using linear search
    CostFunction *cfunc = &cfuncED; // Let Euclidean distance be the cost function
    PoseOptimizer *popt = &poptSD;  // Optimization using steepest descent method
    LoopDetector *lpd = &lpdDM;     //Dummy LoopDetector

    popt->setCostFunction(cfunc);
    poest.setDataAssociator(dass);
    poest.setPoseOptimizer(popt);
    pfu.setDataAssociator(dass);

    smat.setPointCloudMap(pcmap);
    smat.setRefScanMaker(rsm);
    sfront->setLoopDetector(lpd);
    sfront->setDgCheck(false);
    sfront->setPointCloudMap(pcmap);
}

void FrameworkCustomizer::customizeB()
{
    pcmap = &pcmapGT;
    RefScanMaker *rsm = &rsmLM;
    DataAssociator *dass = &dassLS;
    CostFunction *cfunc = &cfuncED;
    PoseOptimizer *popt = &poptSD;
    LoopDetector *lpd = &lpdDM; 

    popt->setCostFunction(cfunc);
    poest.setDataAssociator(dass);
    poest.setPoseOptimizer(popt);
    pfu.setDataAssociator(dass);
    
    smat.setPointCloudMap(pcmap);
    smat.setRefScanMaker(rsm);
    sfront->setLoopDetector(lpd);
    sfront->setPointCloudMap(pcmap);
    sfront->setDgCheck(false);
    

}

void FrameworkCustomizer::customizeC()
{
    pcmap = &pcmapGT;
    RefScanMaker *rsm = &rsmLM;
    DataAssociator *dass = &dassLS;
    CostFunction *cfunc = &cfuncED;
    PoseOptimizer *popt = &poptSL;
    LoopDetector *lpd = &lpdDM; 

    popt->setCostFunction(cfunc);
    poest.setDataAssociator(dass);
    poest.setPoseOptimizer(popt);
    pfu.setDataAssociator(dass);
    
    smat.setPointCloudMap(pcmap);
    smat.setRefScanMaker(rsm);
    sfront->setLoopDetector(lpd);
    sfront->setPointCloudMap(pcmap);
    sfront->setDgCheck(false);

}

void FrameworkCustomizer::customizeD()
{
    pcmap = &pcmapGT;
    RefScanMaker *rsm = &rsmLM;
    DataAssociator *dass = &dassGT;
    CostFunction *cfunc = &cfuncED;
    PoseOptimizer *popt = &poptSL;
    LoopDetector *lpd = &lpdDM; 

    popt->setCostFunction(cfunc);
    poest.setDataAssociator(dass);
    poest.setPoseOptimizer(popt);
    pfu.setDataAssociator(dass);
    
    smat.setPointCloudMap(pcmap);
    smat.setRefScanMaker(rsm);
    sfront->setLoopDetector(lpd);
    sfront->setPointCloudMap(pcmap);
    sfront->setDgCheck(false);
}

void FrameworkCustomizer::customizeE()
{
    pcmap = &pcmapGT;
    RefScanMaker *rsm = &rsmLM;
    DataAssociator *dass = &dassGT;
    CostFunction *cfunc = &cfuncED;
    PoseOptimizer *popt = &poptSL;
    LoopDetector *lpd = &lpdDM; 

    popt->setCostFunction(cfunc);
    poest.setDataAssociator(dass);
    poest.setPoseOptimizer(popt);
    pfu.setDataAssociator(dass);
    
    
    smat.setPointCloudMap(pcmap);
    smat.setRefScanMaker(rsm);
    smat.setScanPointResampler(&spres);
    sfront->setLoopDetector(lpd);
    sfront->setPointCloudMap(pcmap);
    sfront->setDgCheck(false);
}

void FrameworkCustomizer::customizeF()
{
    pcmap = &pcmapGT;
    RefScanMaker *rsm = &rsmLM;
    DataAssociator *dass = &dassLS;
    CostFunction *cfunc = &cfuncPD;
    PoseOptimizer *popt = &poptSL;
    LoopDetector *lpd = &lpdDM; 

    popt->setCostFunction(cfunc);
    poest.setDataAssociator(dass);
    poest.setPoseOptimizer(popt);
    pfu.setDataAssociator(dass);
    
    smat.setPointCloudMap(pcmap);
    smat.setRefScanMaker(rsm);
    smat.setScanPointAnalyser(&spana);
    sfront->setLoopDetector(lpd);
    sfront->setPointCloudMap(pcmap);
    sfront->setDgCheck(false);
}

void FrameworkCustomizer::customizeG()
{
    pcmap = &pcmapGT;
    RefScanMaker *rsm = &rsmLM;
    DataAssociator *dass = &dassGT;
    CostFunction *cfunc = &cfuncPD;
    PoseOptimizer *popt = &poptSL;
    LoopDetector *lpd = &lpdDM; 

    popt->setCostFunction(cfunc);
    poest.setDataAssociator(dass);
    poest.setPoseOptimizer(popt);
    pfu.setDataAssociator(dass);
    
    smat.setPointCloudMap(pcmap);
    smat.setRefScanMaker(rsm);
    smat.setScanPointResampler(&spres);
    smat.setScanPointAnalyser(&spana);
    sfront->setLoopDetector(lpd);
    sfront->setPointCloudMap(pcmap);
    sfront->setDgCheck(false);
}

void FrameworkCustomizer::customizeH()
{
    pcmap = &pcmapLP;
    RefScanMaker *rsm = &rsmLM;
    DataAssociator *dass = &dassLS;
    CostFunction *cfunc = &cfuncPD;
    PoseOptimizer *popt = &poptSL;
    LoopDetector *lpd = &lpdSS; 

    popt->setCostFunction(cfunc);
    poest.setDataAssociator(dass);
    poest.setPoseOptimizer(popt);
    pfu.setDataAssociator(dass);
    
    smat.setPointCloudMap(pcmap);
    smat.setRefScanMaker(rsm);
    smat.setScanPointResampler(&spres);
    smat.setScanPointAnalyser(&spana);
    sfront->setLoopDetector(lpd);
    sfront->setPointCloudMap(pcmap);
    sfront->setDgCheck(true);
}

void FrameworkCustomizer::customizeI()
{
    pcmap = &pcmapLP;
    RefScanMaker *rsm = &rsmLM;
    DataAssociator *dass = &dassLS;
    CostFunction *cfunc = &cfuncPD;
    PoseOptimizer *popt = &poptSL;
    LoopDetector *lpd = &lpdSS; 

    popt->setCostFunction(cfunc);
    poest.setDataAssociator(dass);
    poest.setPoseOptimizer(popt);
    pfu.setDataAssociator(dass);
    
    //path.setPointCloudMap(pcmap);
    smat.setPointCloudMap(pcmap);
    smat.setRefScanMaker(rsm);
    smat.setScanPointResampler(&spres);
    smat.setScanPointAnalyser(&spana);
    sfront->setLoopDetector(lpd);
    sfront->setPointCloudMap(pcmap);
    sfront->setDgCheck(true);
    //sfront->setPathPlanner(&path);
    
}