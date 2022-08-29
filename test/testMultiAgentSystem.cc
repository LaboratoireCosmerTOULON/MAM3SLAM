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

    return 0;
}