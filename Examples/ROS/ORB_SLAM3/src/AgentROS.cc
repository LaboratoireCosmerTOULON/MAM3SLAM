#include "AgentROS.h"

namespace ORB_SLAM3 
{

AgentROS::AgentROS(const string &strSettingsFile, MultiAgentSystem* pMultiAgentSystem, const int initFr, const string &strSequence) : Agent(strSettingsFile, pMultiAgentSystem, initFr, strSequence) {}


void AgentROS::imCallback(const sensor_msgs::ImageConstPtr& msg);
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        // TO-DO JU : SI DEJA GRAYSCALE METTRE L'ARGUMENT AVEC MONO8, SINON NON. AJOUTER PARAMÉTRAGE DE CE TRUC.
        if (this -> is_img_mono) {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        } else {
            std::cout << "case RGB ok" << std::endl;
            cv_ptr = cv_bridge::toCvShare(msg);
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    {
        unique_lock<mutex> lock(mMutexNewFrame);
        cv::Mat im = cv_ptr->image.clone();
        this -> mIm = im;
        this -> mTimestamp = cv_ptr->header.stamp.toSec();
        this -> mGotNewFrame = true;
    }
}