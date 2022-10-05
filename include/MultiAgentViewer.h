#ifndef MULTI_AGENT_VIEWER_H
#define MULTI_AGENT_VIEWER_H

#include "AgentViewer.h"

#include <mutex>

namespace ORB_SLAM3
{

class AgentViewer;

class MultiAgentViewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void AddAgentViewer(AgentViewer* pNewAgentViewer);
    void Run();

private:
    std::vector<AgentViewer*> mvpAgentViewers;

};

}

#endif