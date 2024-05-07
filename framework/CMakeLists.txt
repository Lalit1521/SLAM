PROJECT(framework)

cmake_minimum_required(VERSION 2.8)

find_package(Boost REQUIRED)

find_package(Eigen3)
IF(NOT EIGEN3_INCLUDE_DIR)
  set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_ROOT_DIR})
ENDIF() 

SET(fw_HDRS
    MyUtil.h
    LPoint2D.h
    Pose2D.h
    Scan2D.h
    PointCloudMap.h
    SensorDataReader.h
    RefScanMaker.h
    PoseEstimatorICP.h
    PoseOptimizer.h
    CostFunction.h
    ScanMatcher2D.h
    DataAssociator.h
    SlamFrontEnd.h
    NNGridTable.h
    ScanPointResampler.h
    ScanPointAnalyser.h
    CovarianceCalculator.h
    PoseFuser.h
    LoopDetector.h
    PoseGraph.h
    P2oDriver2D.h
    SlamBackEnd.h
    PriorityQueue.h
    SearchNode.h
    PathPlanning.h
    MotionPlanning.h
)

SET(fw_SRCS
    MyUtil.cpp
    Pose2D.cpp
    Scan2D.cpp
    SensorDataReader.cpp
    PoseEstimatorICP.cpp
    ScanMatcher2D.cpp
    SlamFrontEnd.cpp
    NNGridTable.cpp
    ScanPointResampler.cpp
    ScanPointAnalyser.cpp
    CovarianceCalculator.cpp
    PoseFuser.cpp
    LoopDetector.cpp
    PoseGraph.cpp
    P2oDriver2D.cpp
    SlamBackEnd.cpp
    PriorityQueue.cpp
    SearchNode.cpp
    PathPlanning.cpp
    MotionPlanning.cpp
)

include_directories(
    ${BOOST_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../p2o  
)

link_directories(    
)

ADD_LIBRARY(framework ${fw_SRCS} ${fw_HDRS})

