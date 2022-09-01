#include "Agent.h"

namespace ORB_SLAM3
{

long unsigned int Agent::nNextId=0;

Agent::Agent(const string &strSettingsFile, MultiAgentSystem* pMultiAgentSystem, const int initFr, const string &strSequence) : mpViewer(static_cast<Viewer*>(NULL)),mpMultiAgentSystem(pMultiAgentSystem) 
{
    mnId=nNextId++;

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
    // TO-DO : make Run() perform Tracking
    mptTracking = new thread(&ORB_SLAM3::Agent::Run,this);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(pAtlas, true, false, strSequence);
    // TO-DO : check Run()
    mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run,mpLocalMapper);

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
    LoopClosing* pLoopCloser = mpMultiAgentSystem -> getLoopCloser();
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(pLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(pLoopCloser);

    //Initialize the Viewer thread and launch
    mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile,settings_);
    // TO-DO : make sure it is working
    mptViewer = new thread(&Viewer::Run, mpViewer);
    mpTracker->SetViewer(mpViewer);
    // mpLoopCloser->mpViewer = mpViewer;
    mpViewer->both = mpFrameDrawer->both;

}

Agent::~Agent() {
    Shutdown();
}

void Agent::Run() {
    while(!mbShutDown) {
        if (CheckNewFrame()) {
            {
                unique_lock<mutex> lock(mMutexNewFrame);
                this -> mGotNewFrame = false;
            }
            this -> TrackMonocular(this -> mIm,  this -> mTimestamp);
        }
    }
}

bool Agent::CheckNewFrame() {
    unique_lock<mutex> lock(mMutexNewFrame);
    return mGotNewFrame;
}

Sophus::SE3f Agent::TrackMonocular(const cv::Mat &im, const double &timestamp) 
{
    cout << "ok1" << endl;
    vector<IMU::Point> vImuMeas = vector<IMU::Point>();
    string filename="";
    cout << "ok2" << endl;
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbShutDown)
            return Sophus::SE3f();
    }
    cout << "ok3" << endl;
    if(mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << endl;
        exit(-1);
    }
    cout << "ok4" << endl;
    cv::Mat imToFeed = mIm.clone();
    cout << "ok4bis" << endl;
    if(settings_ && settings_->needToResize()){
        cout << "ok4ter" << endl;
        cv::Mat resizedIm;
        cv::resize(im,resizedIm,settings_->newImSize());
        imToFeed = resizedIm;
    }
    cout << "ok5" << endl;
    Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(imToFeed,timestamp,filename);
    cout << "ok6" << endl;
    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    cout << "ok7" << endl;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    cout << "ok8" << endl;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    cout << "ok9" << endl;
    return Tcw;
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

void Agent::Shutdown() {
    {
        unique_lock<mutex> lock(mMutexReset);
        mbShutDown = true;
    } // mutex automatically released here
    mpLocalMapper->RequestFinish();
    mpViewer->RequestFinish();
    cout << "Shutdown Agent " << mnId << endl;
}

}