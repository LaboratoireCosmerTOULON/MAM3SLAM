#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include"../include/MultiAgentSystem.h"

using namespace std;

int main(int argc, char **argv) {

    // Constructor
    std::string pathToVoc("../Vocabulary/ORBvoc.txt");
    ORB_SLAM3::MultiAgentSystem mas(pathToVoc);

    // void MultiAgentSystem::addAgent(const string &strSettingsFile)
    std::string strSettingsFile1("./settingsForTest_00.yaml");
    std::string strSettingsFile2("./settingsForTest_01.yaml");
    mas.addAgent(strSettingsFile1);
    mas.addAgent(strSettingsFile2);

    return 0;
}