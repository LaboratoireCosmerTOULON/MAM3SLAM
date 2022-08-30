#include "Agent.h"

namespace ORB_SLAM3
{

Agent::Agent(const string &strSettingsFile, MultiAgentSystem* pMultiAgentSystem, const int initFr, const string &strSequence) : mpViewer(static_cast<Viewer*>(NULL)),mpMultiAgentSystem(pMultiAgentSystem) {

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    cv::FileNode node = fsSettings["File.version"];
    if(!node.empty() && node.isString() && node.string() == "1.0"){
        settings_ = new Settings(strSettingsFile,mSensor);
    }
    else{
        cerr << "Error in loading settings" << endl;
        exit(-1);
    }

    Atlas* pAtlas = mpMultiAgentSystem -> getAtlas();
    ORBVocabulary* pVoc = mpMultiAgentSystem -> getVocabulary();
    KeyFrameDatabase* pKFDB = mpMultiAgentSystem -> getKeyFrameDatabase();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(pAtlas);
    mpMapDrawer = new MapDrawer(pAtlas, strSettingsFile, settings_);

    //Initialize the Tracking thread
    cout << "Seq. Name: " << strSequence << endl;
    mpTracker = new Tracking(this, pVoc, mpFrameDrawer, mpMapDrawer, pAtlas, pKFDB, strSettingsFile, mSensor, settings_, strSequence);
    // TO-DO : check Run()
    // mptTracking = new thread(&ORB_SLAM3::System::Run,this);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(pAtlas, true, false, strSequence);
    // TO-DO : check Run()
    // mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run,mpLocalMapper);

    mpLocalMapper->mInitFr = initFr;
    if(settings_)
        mpLocalMapper->mThFarPoints = settings_->thFarPoints();
    else
        mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
    if(mpLocalMapper->mThFarPoints!=0)
    {
        cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
        mpLocalMapper->mbFarPoints = true;
    }
    else
        mpLocalMapper->mbFarPoints = false;

    //Set pointers between threads
    // TO-DO : make sure it is working
    mpTracker->SetLocalMapper(mpLocalMapper);
    // mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    // mpLocalMapper->SetLoopCloser(mpLoopCloser);

    //Initialize the Viewer thread and launch
    // TO-DO : make sure it is working
    // mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile,settings_);
    // mptViewer = new thread(&Viewer::Run, mpViewer);
    // mpTracker->SetViewer(mpViewer);
    // mpLoopCloser->mpViewer = mpViewer;
    // mpViewer->both = mpFrameDrawer->both;

}
    
void Agent::Run() {}

bool Agent::CheckNewFrame() {}

Sophus::SE3f Agent::TrackMonocular(const cv::Mat &im, const double &timestamp) {
    return Sophus::SE3f();
}

void Agent::ResetActiveMap() {}

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