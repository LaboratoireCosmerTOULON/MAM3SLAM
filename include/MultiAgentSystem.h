#ifndef MULTI_AGENT_SYSTEM_H
#define MULTI_AGENT_SYSTEM_H

#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Atlas.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
// #include "Viewer.h"
#include "MultiAgentViewer.h"
#include "ImuTypes.h"
#include "Settings.h"

#include "Agent.h"


namespace ORB_SLAM3
{

class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;

public:
    static void PrintMess(std::string str, eLevel lev)
    {
        if(lev <= th)
            cout << str << endl;
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};

// class Viewer;
class Atlas;
class LoopClosing;
class Settings;
class Agent;
class MultiAgentViewer;

class MultiAgentSystem 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MultiAgentSystem(const string &strVocFile, bool bActiveLC = true);
    ~MultiAgentSystem();
    void addAgent(const string &strSettingsFile);
    void addAgent(Agent* pNewAgent);

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();
    bool isShutDown();

    ORBVocabulary* getVocabulary();
    KeyFrameDatabase* getKeyFrameDatabase();
    Atlas* getAtlas();
    LoopClosing* getLoopCloser();

    // FIXME !!! Pratique à risque.
    Agent* getAgent(int i);

    void StartViewer(); // WARNING : adding new agents after starting viewer may lead to core dumped

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

    // System threads: Loop Closing.
    std::thread* mptLoopClosing;

    // Agents
    std::vector<Agent*> mvpAgents;

    // Reset flag
    std::mutex mMutexReset;

    // Shutdown flag
    bool mbShutDown;

    string mStrVocabularyFilePath;

    // MultiAgentViewer anf thread
    MultiAgentViewer* mpMultiAgentViewer;
    std::thread* mptMultiAgentViewer;
};

}

#endif