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
#include "Viewer.h"
#include "ImuTypes.h"
#include "Settings.h"

#include "MultiAgentSystem.h"

namespace ORB_SLAM3 {

class Agent
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Agent(const string &strSettingsFile);
    
    // Main function
    void Run();
    bool CheckNewFrame();

    bool mGotNewFrame = false;
    cv::Mat mIm;
    double mTimestamp;
    std::mutex mMutexNewFrame;

    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

private:

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // System threads: Local Mapping, Tracking.
    std::thread* mptLocalMapping;
    std::thread* mptTracking;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;

    Settings* settings_;

    MultiAgentSystem* mpMultiAgentSystem;

};

}

#endif