#ifndef AGENT_ROS_H
#define AGENT_ROS_H

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/Agent.h"

namespace ORB_SLAM3 
{

class AgentROS : public Agent 
{
protected:

    ros::NodeHandle mNh; // ros nodehandle
    std::string mImTopicName; // image topic name
    ros::Subscriber mImSubscriber; // image subscriber


public:

    bool is_img_mono;

    AgentROS(const string &strSettingsFile, MultiAgentSystem* pMultiAgentSystem, const int initFr = 0, const string &strSequence = std::string());

    // Main function
    void Run();

    void setImgSub(std::string &imgTopicName);

    void imCallback(const sensor_msgs::ImageConstPtr& msg);
};
}

#endif