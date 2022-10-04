/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/Agent.h"
// #include"AgentROS.h"

using namespace std;

class ImageGrabber {
    public:
        ImageGrabber(ORB_SLAM3::Agent* pAgent, bool is_img_mono):mpAgent(pAgent), is_img_mono(is_img_mono){}

        void GrabImage(const sensor_msgs::ImageConstPtr& msg);

        ORB_SLAM3::Agent* mpAgent;

        bool is_img_mono;

        std::mutex mMutexNewFrame;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3 && argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings [is_mono]" << endl;        
        ros::shutdown();
        return 1;
    }    

    bool is_img_mono = false;
    if (argc == 4) {
        std::string is_img_mono_str(argv[3]);
        is_img_mono = is_img_mono_str == "true";
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::MultiAgentSystem mas(argv[1]);
    std::string strSettingsFile1("/home/ju/Copie_de_travail_ORBSLAM3/ORB_SLAM3/test/settingsForTest_00.yaml");
    std::string strSettingsFile2("/home/ju/Copie_de_travail_ORBSLAM3/ORB_SLAM3/test/settingsForTest_01.yaml");
    mas.addAgent(strSettingsFile1);
    mas.addAgent(strSettingsFile2);
    ImageGrabber igb1(mas.getAgent(0), is_img_mono);
    ImageGrabber igb2(mas.getAgent(1), is_img_mono);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub1 = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb1);
    ros::Subscriber sub2 = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb2);
    
    ros::MultiThreadedSpinner s(0);
    ros::spin();

    // // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ORB_SLAM3::MultiAgentSystem mas(argv[1]);
    // ORB_SLAM3::Agent jeanPhilippe(argv[2], &mas);
    // ImageGrabber igb1(&jeanPhilippe, is_img_mono);

    // ros::NodeHandle nodeHandler;
    // ros::Subscriber sub1 = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb1);
    
    // ros::MultiThreadedSpinner s(0);
    // ros::spin();

    // // Stop all threads
    // jeanPhilippe.Shutdown();
    

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
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

    // mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    {
        unique_lock<mutex> lock(mMutexNewFrame);
        cv::Mat im = cv_ptr->image.clone();
        mpAgent -> mIm = im;
        mpAgent -> mTimestamp = cv_ptr->header.stamp.toSec();
        mpAgent -> mGotNewFrame = true;
    }
}
