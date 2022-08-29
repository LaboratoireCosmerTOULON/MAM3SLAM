#ifndef MULTI_AGENT_SYSTEM_H
#define MULTI_AGENT_SYSTEM_H

#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Atlas.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "ImuTypes.h"
#include "Settings.h"

#include "Agent.h"


namespace ORB_SLAM3
{

class Viewer;
class FrameDrawer;
class MapDrawer;
class Atlas;
class LoopClosing;
class Settings;
class Agent;

class MultiAgentSystem 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MultiAgentSystem(const string &strVocFile);
    void addAgent(const string &strSettingsFile);

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();
    bool isShutDown();

private:
    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    //Map* mpMap;
    Atlas* mpAtlas;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Loop Closing and Viewer.
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Agents
    std::vector<Agent*> mAgents;

    // Shutdown flag
    bool mbShutDown;

    string mStrVocabularyFilePath;

    Settings* settings_;

};

}

#endif