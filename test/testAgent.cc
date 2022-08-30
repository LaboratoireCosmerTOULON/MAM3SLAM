#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include"../include/Agent.h"

using namespace std;

int main(int argc, char **argv) {

    // Constructor
    std::string strSettingsFile("./settingsForTest.yaml");
    std::string pathToVoc("../Vocabulary/ORBvoc.txt");
    ORB_SLAM3::MultiAgentSystem mas(pathToVoc);
    ORB_SLAM3::Agent jeanPhilippe(strSettingsFile, &mas);


    return 0;
}