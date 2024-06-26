#ifndef AGENT_H
#define AGENT_H

#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "LocalMapping.h"
// #include "Viewer.h"
#include "AgentViewer.h"
#include "ImuTypes.h"
#include "Settings.h"

#include "MultiAgentSystem.h"
#include "Map.h"

namespace MAM3SLAM {

class FrameDrawer;
class MapDrawer;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;
class MultiAgentSystem;
class AgentViewer;


class Agent
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        IMU_MONOCULAR=3,
        IMU_STEREO=4,
        IMU_RGBD=5,
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Agent(const string &strSettingsFile, MultiAgentSystem* pMultiAgentSystem, bool bUseViewer = true, const int initFr = 0, const string &strSequence = std::string());
    ~Agent();

    // Main function
    void Run();
    bool CheckNewFrame();

    bool mGotNewFrame = false;
    cv::Mat mIm;
    cv::Mat mIm2;
    double mTimestamp;
    std::mutex mMutexNewFrame;

    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackMonocular(const cv::Mat &im, const double &timestamp);

    // Reset the active map
    void ResetActiveMap();

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    static long unsigned int nNextId;
    long unsigned int mnId;

    void Shutdown();

    AgentViewer* getAgentViewer();

    // current map
    void SetCurrentMap(Map* newActiveMap);
    Map* GetCurrentMap(bool doLock = true);

    // loop closure data
    bool mbLoopDetected;
    int mnLoopNumCoincidences;
    int mnLoopNumNotFound;
    KeyFrame* mpLoopLastCurrentKF;
    g2o::Sim3 mg2oLoopSlw;
    g2o::Sim3 mg2oLoopScw;
    KeyFrame* mpLoopMatchedKF;
    std::vector<MapPoint*> mvpLoopMPs;
    std::vector<MapPoint*> mvpLoopMatchedMPs;
    bool mbMergeDetected;
    int mnMergeNumCoincidences;
    int mnMergeNumNotFound;
    KeyFrame* mpMergeLastCurrentKF;
    g2o::Sim3 mg2oMergeSlw;
    g2o::Sim3 mg2oMergeSmw;
    g2o::Sim3 mg2oMergeScw;
    KeyFrame* mpMergeMatchedKF;
    std::vector<MapPoint*> mvpMergeMPs;
    std::vector<MapPoint*> mvpMergeMatchedMPs;
    std::vector<KeyFrame*> mvpMergeConnectedKFs;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    void SaveTrackingStates();
    void SaveTrajectory();

    // Real-time analysis
    void SaveTimes();

private:

    // Input sensor
    eSensor mSensor = MONOCULAR;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    // Viewer* mpViewer;
    AgentViewer* mpAgentViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Tracking, Viewer.
    std::thread* mptLocalMapping;
    std::thread* mptTracking;
    std::thread* mptAgentViewer;
    // std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbResetActiveMap;

    std::mutex mMutexMap;

    // Shutdown flag
    bool mbShutDown = false;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;

    Settings* settings_;

    MultiAgentSystem* mpMultiAgentSystem;

    // Current current map
    Map* mpCurrentMap;

};

}

#endif