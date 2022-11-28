#include "MultiAgentSystem.h"

namespace ORB_SLAM3
{

Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

MultiAgentSystem::MultiAgentSystem(const string &strVocFile, bool bActiveLC, bool bUseViewer) : mbShutDown(false), mbUseViewer(bUseViewer) {

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
    mpAtlas = new Atlas();

    //Initialize the Loop Closing thread and launch
    // mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR
    bool bFixScale = false;
    mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, bFixScale, bActiveLC);
    mpLoopCloser->SetMultiAgentSystem(this);
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
    Agent* pNewAgent = new Agent(strSettingsFile, this, mbUseViewer);
    mvpAgents.push_back(pNewAgent);
    usleep(3000);
}

void MultiAgentSystem::addAgent(Agent* pNewAgent) {
    mvpAgents.push_back(pNewAgent);
    usleep(3000);
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

void MultiAgentSystem::StartViewer() {
       // MultiAgentViewer
    mpMultiAgentViewer = new MultiAgentViewer();
    for (int i = 0 ; i < mvpAgents.size() ; i++) {
        mpMultiAgentViewer -> AddAgentViewer(mvpAgents[i] -> getAgentViewer());
    }
    mptMultiAgentViewer = new thread(&ORB_SLAM3::MultiAgentViewer::Run, mpMultiAgentViewer);
}

std::vector<Agent*> MultiAgentSystem::GetAgentsInMap(long unsigned int nMapId)
{
    std::vector<Agent*> vpAgentsInMap;
    for(Agent* pAgent : mvpAgents)
    {
        if (pAgent->GetCurrentMap()->GetId() == nMapId)
        {
            vpAgentsInMap.push_back(pAgent);
        }
    }
    return vpAgentsInMap;
}

}