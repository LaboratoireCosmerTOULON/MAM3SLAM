#include "MultiAgentSystem.h"

namespace ORB_SLAM3
{

MultiAgentSystem::MultiAgentSystem(const string &strVocFile) : mpViewer(static_cast<Viewer*>(NULL)), mbShutDown(false) {

    mStrVocabularyFilePath = strVocFile;

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Atlas
    cout << "Initialization of Atlas from scratch " << endl;
    mpAtlas = new Atlas(0);

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpAtlas);
    // TODO : deal with the settings
    // mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);

    //Initialize the Loop Closing thread and launch
    // mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR
    mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, true, true);
    mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);
}

MultiAgentSystem::~MultiAgentSystem() {
    mpLoopCloser->RequestFinish();
}

void MultiAgentSystem::addAgent(const string &strSettingsFile) {}

bool MultiAgentSystem::MapChanged() {
    return true;
}

void MultiAgentSystem::Shutdown() {}

bool MultiAgentSystem::isShutDown() {
    return true;
}

}