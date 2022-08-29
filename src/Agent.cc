#include "Agent.h"

namespace ORB_SLAM3
{

Agent::Agent(const string &strSettingsFile) {}
    
void Agent::Run() {}

bool Agent::CheckNewFrame() {}

Sophus::SE3f Agent::TrackMonocular(const cv::Mat &im, const double &timestamp) {
    return Sophus::SE3f();
}

int Agent::GetTrackingState() {
    return 0;
}

std::vector<MapPoint*> Agent::GetTrackedMapPoints() {
    std::vector<MapPoint*> foo;
    return foo;
}

std::vector<cv::KeyPoint> Agent::GetTrackedKeyPointsUn() {
    std::vector<cv::KeyPoint> foo;
    return foo;
}

}