#include "MultiAgentViewer.h"

namespace ORB_SLAM3
{

void MultiAgentViewer::AddAgentViewer(AgentViewer* pNewAgentViewer) {
    mvpAgentViewers.push_back(pNewAgentViewer);
}

void MultiAgentViewer::Run()
{
    while(1)
    {
        for (int i = 0 ; i < mvpAgentViewers.size() ; i++)
        {
            mvpAgentViewers[i] -> Update();
            cv::waitKey(1);
        }
    }
}

}