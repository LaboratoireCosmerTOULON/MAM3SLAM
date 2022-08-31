#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include"../include/Agent.h"

using namespace std;

int main(int argc, char **argv) {

    // Constructor
    std::string strSettingsFile("./settingsForTest_00.yaml");
    std::string pathToVoc("../Vocabulary/ORBvoc.txt");
    ORB_SLAM3::MultiAgentSystem mas(pathToVoc);
    ORB_SLAM3::Agent jeanPhilippe(strSettingsFile, &mas);

    std::mutex mMutexNewFrame;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    {
        unique_lock<mutex> lock(mMutexNewFrame);
        jeanPhilippe.mGotNewFrame = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    return 0;
}