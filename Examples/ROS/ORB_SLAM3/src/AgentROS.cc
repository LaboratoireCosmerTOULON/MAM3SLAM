#include "AgentROS.h"

namespace ORB_SLAM3 
{

AgentROS::AgentROS(const string &strSettingsFile, MultiAgentSystem* pMultiAgentSystem, const int initFr, const string &strSequence) : Agent(strSettingsFile, pMultiAgentSystem, initFr, strSequence) {}

}