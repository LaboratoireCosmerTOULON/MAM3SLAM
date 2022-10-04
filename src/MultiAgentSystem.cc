#include "MultiAgentSystem.h"

namespace ORB_SLAM3
{

MultiAgentSystem::MultiAgentSystem(const string &strVocFile) : mbShutDown(false) {

    mStrVocabularyFilePath = strVocFile;

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Atlas
    cout << "Initialization of Atlas from scratch " << endl;
    mpAtlas = new Atlas(0);

    //Initialize the Loop Closing thread and launch
    // mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR
    bool activeLC = false;
    mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, true, activeLC);
    mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

    // // Fix verbosity
    // Verbose::SetTh(Verbose::VERBOSITY_QUIET);
}

MultiAgentSystem::~MultiAgentSystem() {
    Shutdown();
    for (int i = 0 ; i < mvpAgents.size() ; i++) {
        mvpAgents[i] -> Shutdown();
    }
}

void MultiAgentSystem::addAgent(const string &strSettingsFile) {
    Agent* pNewAgent = new Agent(strSettingsFile, this);
    mvpAgents.push_back(pNewAgent);
}

void MultiAgentSystem::addAgent(Agent* pNewAgent) {
    mvpAgents.push_back(pNewAgent);
}

bool MultiAgentSystem::MapChanged() {
    static int n=0;
    int curn = mpAtlas->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void MultiAgentSystem::Shutdown() {
    {
        unique_lock<mutex> lock(mMutexReset);
        mbShutDown = true;
    }
    mpLoopCloser->RequestFinish();
    cout << "Shutdown MultiAgentsystem" << endl;
}


bool MultiAgentSystem::isShutDown() {
    return true;
}

ORBVocabulary* MultiAgentSystem::getVocabulary() {
    return mpVocabulary;
}

KeyFrameDatabase* MultiAgentSystem::getKeyFrameDatabase() {
    return mpKeyFrameDatabase;
}

Atlas* MultiAgentSystem::getAtlas() {
    return mpAtlas;
}

LoopClosing* MultiAgentSystem::getLoopCloser() {
    return mpLoopCloser;
}

// FIXME !!! Pratique à risque.
Agent* MultiAgentSystem::getAgent(int i) {
    return mvpAgents[i];
}

}