#include "AgentViewer.h"

namespace ORB_SLAM3
{

AgentViewer::AgentViewer(Agent* pAgent, FrameDrawer* pFrameDrawer, Tracking *pTracking, const string &strSettingPath, Settings* settings) : mpAgent(pAgent), mpFrameDrawer(pFrameDrawer), mpTracker(pTracking), mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    // Load settings
    if(settings){
        newParameterLoader(settings);
    }
    else{

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        bool is_correct = ParseViewerParamFile(fSettings);

        if(!is_correct)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }
    mbStopTrack = false;

    // OpenCV window
    mCurrentFrameWindowName = "ORB-SLAM3: Current Frame - Agent " + to_string(mpAgent -> mnId);
    cv::namedWindow(mCurrentFrameWindowName);
    mtrackedImageScale = mpTracker->GetImageScale();
}

void AgentViewer::newParameterLoader(Settings *settings) 
{
    mImageViewerScale = 1.f;

    float fps = settings->fps();
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::Size imSize = settings->newImSize();
    mImageHeight = imSize.height;
    mImageWidth = imSize.width;

    mImageViewerScale = settings->imageViewerScale();

}

bool AgentViewer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    mImageViewerScale = 1.f;

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::FileNode node = fSettings["Camera.width"];
    if(!node.empty())
    {
        mImageWidth = node.real();
    }
    else
    {
        std::cerr << "*Camera.width parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Camera.height"];
    if(!node.empty())
    {
        mImageHeight = node.real();
    }
    else
    {
        std::cerr << "*Camera.height parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.imageViewScale"];
    if(!node.empty())
    {
        mImageViewerScale = node.real();
    }

    return !b_miss_params;
}

void AgentViewer::Update()
{
    cv::Mat toShow = mpFrameDrawer->DrawFrame(mtrackedImageScale);
    if(mImageViewerScale != 1.f)
    {
        int width = toShow.cols * mImageViewerScale;
        int height = toShow.rows * mImageViewerScale;
        cv::resize(toShow, toShow, cv::Size(width, height));
    }
    cv::imshow(mCurrentFrameWindowName,toShow);
}

}