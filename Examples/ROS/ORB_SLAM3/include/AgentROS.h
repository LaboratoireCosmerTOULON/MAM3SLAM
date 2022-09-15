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
public:
    AgentROS(const string &strSettingsFile, MultiAgentSystem* pMultiAgentSystem, const int initFr = 0, const string &strSequence = std::string());
};
}

#endif