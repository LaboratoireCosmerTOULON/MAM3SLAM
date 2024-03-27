#include "MultiAgentViewer.h"

namespace MAM3SLAM
{

void MultiAgentViewer::AddAgentViewer(AgentViewer* pNewAgentViewer) {
    mvpAgentViewers.push_back(pNewAgentViewer);
}

void MultiAgentViewer::Run()
{
    // Create tracking windows for all agents
    for (int i = 0 ; i < mvpAgentViewers.size() ; i++)
        {
            mvpAgentViewers[i] -> CreateFrameWindow();
        }
    while(1)
    {
        // Update tracking windows for all agents
        for (int i = 0 ; i < mvpAgentViewers.size() ; i++)
        {
            mvpAgentViewers[i] -> UpdateCurrentFrameWindow();
            cv::waitKey(1);
        }
    }
}

}