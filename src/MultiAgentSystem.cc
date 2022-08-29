#include "MultiAgentSystem.h"

namespace ORB_SLAM3
{

MultiAgentSystem::MultiAgentSystem(const string &strVocFile) {}

void MultiAgentSystem::addAgent(const string &strSettingsFile) {}

bool MultiAgentSystem::MapChanged() {
    return true;
}

void MultiAgentSystem::Shutdown() {}

bool MultiAgentSystem::isShutDown() {
    return true;
}

}