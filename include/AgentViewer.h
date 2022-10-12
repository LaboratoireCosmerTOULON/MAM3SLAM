#ifndef AGENT_VIEWER_H
#define AGENT_VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "MultiAgentSystem.h"
#include "Settings.h"

#include <mutex>

namespace ORB_SLAM3
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class Agent;
class Settings;

class AgentViewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    AgentViewer(Agent* pAgent, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath, Settings* settings);

    void newParameterLoader(Settings* settings);

    void CreateFrameWindow();

    void UpdateCurrentFrameWindow();

    void Run();

    bool isStopped();

private:

    bool ParseViewerParamFile(cv::FileStorage &fSettings);

    bool Stop();

    Agent* mpAgent;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;
    float mImageViewerScale;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

    bool mbStopTrack;

    string mMapWindowName;
    string mCurrentFrameWindowName;

    float mtrackedImageScale;

};

}

#endif