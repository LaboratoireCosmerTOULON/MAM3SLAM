#include "Agent.h"

namespace ORB_SLAM3
{

long unsigned int Agent::nNextId=0;

Agent::Agent(const string &strSettingsFile, MultiAgentSystem* pMultiAgentSystem, bool bUseViewer, const int initFr, const string &strSequence) : mpMultiAgentSystem(pMultiAgentSystem), mnLoopNumCoincidences(0), mnLoopNumNotFound(0),
    mbLoopDetected(false), mbMergeDetected(false), mnMergeNumCoincidences(0), mnMergeNumNotFound(0)
{
    mnId=nNextId++; // ID
    mpCurrentMap = static_cast<Map*>(NULL); // current map

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
    mpFrameDrawer = new FrameDrawer(this, pAtlas);
    // std::cout << "Agent " << mnId << " just before map drawer instantiation" << std::endl;
    mpMapDrawer = new MapDrawer(this, pAtlas, strSettingsFile, settings_);
    // std::cout << "Agent " << mnId << " just after map drawer instantiation" << std::endl;

    //Initialize the Tracking thread
    cout << "Seq. Name: " << strSequence << endl;
    mpTracker = new Tracking(this, pVoc, mpFrameDrawer, mpMapDrawer, pAtlas, pKFDB, strSettingsFile, mSensor, settings_, strSequence);
    // TO-DO : make Run() perform Tracking
    mptTracking = new thread(&ORB_SLAM3::Agent::Run,this);
    std::cout << "attached for Agent " << mnId << std::endl;

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(this, pAtlas, true, false, strSequence);
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

    if (bUseViewer)
    {
        // AgentViewer
        // std::cout << "Agent " << mnId << " just before agent viewer instantiation" << std::endl;
        mpAgentViewer = new AgentViewer(this, mpFrameDrawer, mpMapDrawer, mpTracker,strSettingsFile,settings_);
        // std::cout << "Agent " << mnId << " just after agent viewer instantiation" << std::endl;
        mptAgentViewer = new thread(&AgentViewer::Run, mpAgentViewer);
        // std::cout << "Agent " << mnId << " just after agent viewer thread setting" << std::endl;
    }


}

Agent::~Agent()
{
    Shutdown();
}

void Agent::Run()
{
    // std::cout << "Agent " << mnId << " is starting running" << std::endl;
    // std:: cout << "mbShutDown : " << mbShutDown << std::endl;
    while(!mbShutDown) {
        // std::cout << "Agent " << mnId << " is running" << std::endl;
        if (CheckNewFrame()) {
            {
                // unique_lock<mutex> lock(mMutexNewFrame);
                this -> mGotNewFrame = false;
            }
            // std::cout << "Agent " << mnId << " got new frame" << std::endl; // DEBUG
            this -> TrackMonocular(this -> mIm,  this -> mTimestamp);
            usleep(3000);
        }
    }
}

bool Agent::CheckNewFrame()
{
    // unique_lock<mutex> lock(mMutexNewFrame);
    return mGotNewFrame;
}

Sophus::SE3f Agent::TrackMonocular(const cv::Mat &im, const double &timestamp) 
{
    // cout << "ok1" << endl;
    vector<IMU::Point> vImuMeas = vector<IMU::Point>();
    string filename="";
    // cout << "ok2" << endl;
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbShutDown)
            return Sophus::SE3f();
    }
    // cout << "ok3" << endl;
    if(mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << endl;
        exit(-1);
    }
    // cout << "ok4" << endl;
    // cv::Mat imToFeed = mIm.clone();
    cv::Mat imToFeed = mIm;
    // cout << "ok4bis" << endl;
    if(settings_ && settings_->needToResize()){
        // cout << "ok4ter" << endl;
        cv::Mat resizedIm;
        cv::resize(im,resizedIm,settings_->newImSize());
        imToFeed = resizedIm;
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbResetActiveMap)
        {
            cout << "AGENT-> Reseting active map in monocular case" << endl;
            mpTracker->ResetActiveMap(); // FIXME : update fct
            mbResetActiveMap = false;
        }
    }

    // cout << "ok5" << endl;
    Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(imToFeed,timestamp,filename);
    // cout << "ok6" << endl;
    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    // cout << "ok7" << endl;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    // cout << "ok8" << endl;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    // cout << "ok9" << endl;
    // std::cout << "Agent " << mnId << " achieved TrackMonocular" << std::endl; // DEBUG
    return Tcw;
}

void Agent::ResetActiveMap()
{
    std::cout << "Warning : agent::ResetActiveMap() called but not implemented" << std::endl;
    unique_lock<mutex> lock(mMutexReset);
    mbResetActiveMap = true;
} // TODO ?

int Agent::GetTrackingState()
{
    return 0;
}

std::vector<MapPoint*> Agent::GetTrackedMapPoints()
{
    std::vector<MapPoint*> foo;
    return foo;
}

std::vector<cv::KeyPoint> Agent::GetTrackedKeyPointsUn()
{
    std::vector<cv::KeyPoint> foo;
    return foo;
}

void Agent::Shutdown()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbShutDown = true;
    } // mutex automatically released here
    mpLocalMapper->RequestFinish();
    // mpViewer->RequestFinish();
    cout << "Shutdown Agent " << mnId << endl;
}

AgentViewer* Agent::getAgentViewer()
{
    return mpAgentViewer;
}

void Agent::SetCurrentMap(Map* newActiveMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpCurrentMap = newActiveMap;
}

Map* Agent::GetCurrentMap(bool doLock)
{
    if (doLock) {
        unique_lock<mutex> lock(mMutexMap);
        return mpCurrentMap;
    } else {
        return mpCurrentMap;
    }
}

void Agent::SaveTrackingStates()
{
    mpTracker->SaveStates();
}

void Agent::SaveTrajectory()
{
    std::stringstream ss;
    ss << "/home/ju/Copie_de_travail_ORBSLAM3/ORB_SLAM3/output/Trajectory_" << mnId << ".txt";
    std::string filename = ss.str();
    std::cout << "Saving trajectory to " << filename << " ..." << std:: endl;

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    f << setprecision(6) << "ts" << setprecision(7) << " tx ty tz qx qy qz qw agent ref_KF_ts" << endl;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    
    // std::cout << "mpTracker->mvpReferences.size()" << mpTracker->mvpReferences.size() << std::endl;
    // std::cout << "mpTracker->mvFrameTimes.size()" << mpTracker->mvFrameTimes.size() << std::endl;
    // std::cout << "mpTracker->mvbLost.size()" << mpTracker->mvbLost.size() << std::endl;

    int nMaxIter = mpTracker->mvRelativeFramePoses.size();
    int nIter = 0;

    for(int i = 0 ; i < nMaxIter ; i++)
    {
        bool bLost = mpTracker->mvbLost[i];
        double frameTime = mpTracker->mvFrameTimes[i];
        KeyFrame* pKF = mpTracker->mvpReferences[i];
        Sophus::SE3f relativeFramePose = mpTracker->mvRelativeFramePoses[i];

        // std::cout << bLost << std::endl;
        // std::cout << std::fixed;
        // std::cout << frameTime << std::endl;
        // std::cout << pKF->mnId << std::endl;
        // std::cout << relativeFramePose.unit_quaternion().x() << std::endl;

        // std::cout << "Iteration " << nIter++ << " out of " << nMaxIter << std::endl;
        // cout << "1" << endl;
        if(bLost)
            continue;

        // KeyFrame* pKF = mpTracker->mvpReferences[i];

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        // cout << "2.5" << endl;

        // cout << "KF: " << pKF->mnId << endl;
        // std::cout << "pKF is from map " << pKF->GetMap()->GetId() << std::endl;
        // std::cout << "mpTracker->mvRelativeFramePoses[i].unit_quaternion().x : " << mpTracker->mvRelativeFramePoses[i].unit_quaternion().x() << std::endl;
        // std::cout << std::fixed;
        // std::cout << "mpTracker->mvFrameTimes[i]: " << mpTracker->mvFrameTimes[i] << std::endl;

        bool bRefOk = true;
        while(pKF->isBad() && bRefOk)
        {
            if (!(pKF->GetParent() == NULL)) 
            {
                // cout << " 2.bad" << endl;
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
                // cout << "--Parent KF: " << pKF->mnId << endl;
            } 
            else
            {
                bRefOk = false;
            }
        }

        if (!bRefOk)
        {
            // std::cout << "ref pas ok" << std::endl;
            continue;
        }

        // cout << "3" << endl;
        // cout << "KF: " << pKF->mnId << endl;
        // std::cout << "pKF is from map " << pKF->GetMap()->GetId() << std::endl;

        Trw = Trw * pKF->GetPose(); // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference
        // std::cout << "Trw.unit_quaternion().x : " << Trw.unit_quaternion().x() << std::endl;

        // cout << "4" << endl;

        Sophus::SE3f Twc = (relativeFramePose*Trw).inverse();
        // std::cout << "Twc.unit_quaternion().x : " << Twc.unit_quaternion().x() << std::endl;
        Eigen::Quaternionf q = Twc.unit_quaternion();
        Eigen::Vector3f twc = Twc.translation();
        // std::cout << mpTracker->mvFrameTimes[i] << std::endl;
        // std::cout << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        // std::cout << mnId << std::endl;
        // std::cout << pKF->GetMap()->GetId() << std::endl;
        // f << setprecision(6) << frameTime << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " << mnId << " " << pKF->GetMap()->GetId() << endl;
        // int mapId = 0; //pKF->GetMap()->GetId();
        // Map* test = pKF->GetMap();
        // std::cout << test->mnId << std::endl;
        f << setprecision(6) << frameTime << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " << mnId << " " << setprecision(6) << pKF->mTimeStamp << std::endl;

        // cout << "5" << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
    mpLocalMapper->ShowDurationStats();

}
/*{
    std::stringstream ss;
    ss << "/home/ju/Copie_de_travail_ORBSLAM3/ORB_SLAM3/output/Trajectory_" << mnId << ".txt";
    std::string filename = ss.str();
    std::cout << "Saving trajectory to " << filename << " ..." << std:: endl;

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    f << setprecision(6) << "ts" << setprecision(7) << " tx ty tz qx qy qz qw agent map" << endl;

    // vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    // int numMaxKFs = 0;
    // Map* pBiggerMap;
    // std::cout << "There are " << std::to_string(vpMaps.size()) << " maps in the atlas" << std::endl;
    // for(Map* pMap :vpMaps)
    // {
    //     std::cout << "  Map " << std::to_string(pMap->GetId()) << " has " << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs" << std::endl;
    //     if(pMap->GetAllKeyFrames().size() > numMaxKFs)
    //     {
    //         numMaxKFs = pMap->GetAllKeyFrames().size();
    //         pBiggerMap = pMap;
    //     }
    // }

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    std::cout << "mpTracker->mlpReferences.size()" << mpTracker->mlpReferences.size() << std::endl;
    std::cout << "mpTracker->mlFrameTimes.size()" << mpTracker->mlFrameTimes.size() << std::endl;
    std::cout << "mpTracker->mlbLost.size()" << mpTracker->mlbLost.size() << std::endl;

    int nMaxIter = mpTracker->mlRelativeFramePoses.size();
    int nIter = 0;

    for(auto lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        std::cout << "Iteration " << nIter++ << " out of " << nMaxIter << std::endl;
        //cout << "1" << endl;
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        // cout << "2.5" << endl;

        cout << "KF: " << pKF->mnId << endl;
        std::cout << "pKF is from map " << pKF->GetMap()->GetId() << std::endl;
        std::cout << "(*lit).unit_quaternion().x : " << (*lit).unit_quaternion().x() << std::endl;
        std::cout << std::fixed;
        std::cout << "*lT: " << *lT << std::endl;

        bool bRefOk = true;
        while(pKF->isBad() && bRefOk)
        {
            if (!(pKF->GetParent() == NULL)) 
            {
                cout << " 2.bad" << endl;
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
                cout << "--Parent KF: " << pKF->mnId << endl;
            } 
            else
            {
                bRefOk = false;
            }
        }

        if (!bRefOk)
        {
            std::cout << "ref pas ok" << std::endl;
            continue;
        }

        cout << "3" << endl;
        cout << "KF: " << pKF->mnId << endl;
        std::cout << "pKF is from map " << pKF->GetMap()->GetId() << std::endl;

        Trw = Trw * pKF->GetPose(); // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference
        std::cout << "Trw.unit_quaternion().x : " << Trw.unit_quaternion().x() << std::endl;

        cout << "4" << endl;

        Sophus::SE3f Twc = ((*lit)*Trw).inverse();
        std::cout << "Twc.unit_quaternion().x : " << Twc.unit_quaternion().x() << std::endl;
        Eigen::Quaternionf q = Twc.unit_quaternion();
        Eigen::Vector3f twc = Twc.translation();
        std::cout << *lT << std::endl;
        std::cout << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        std::cout << mnId << std::endl;
        std::cout << pKF->GetMap()->GetId() << std::endl;
        // f << setprecision(6) << *lT << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " " << mnId << " " << pKF->GetMap()->GetId() << endl;

        cout << "5" << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;

}*/

}