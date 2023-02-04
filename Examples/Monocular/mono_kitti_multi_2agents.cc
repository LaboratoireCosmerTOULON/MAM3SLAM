/**
* This file is part of iORB-SLAM.
*
* Copyright (C) 2016-2017 Hayyan Daoud <hayyan dot d at gmail dot com> (University of Malaya)
* For more information see <https://github.com/hdaoud/ORBSLAMM>
*
* iORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* iORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORBSLAMM. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"../include/Agent.h"
#include<sys/time.h>
#include<thread>

using namespace std;


void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);
void RunSLAM(int& start, int& nImages, ORB_SLAM3::Agent* agent, vector<string>& vstrImageFilenames, vector<double>& vTimestamps, std::atomic<bool>& done);
void RunSLAM2(int& start, int& nImages, ORB_SLAM3::Agent* agent, vector<string>& vstrImageFilenames, vector<double>& vTimestamps, std::atomic<bool>& done);


int main(int argc, char **argv)
{
    if(argc < 6)
    {
        cerr << endl << "Usage: ./mono_kitti_multi2 path_to_vocabulary path_to_settings_of_sequence1 path_to_sequence1 path_to_settings_of_sequence2 path_to_sequence2 " << endl;
        return 1;
    }

    std::cout << "ok1" << std::endl;

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    std::cout << "ok2" << std::endl;
    
    vector<string> vstrImageFilenames2;
    vector<double> vTimestamps2;
    if(argv[4])
        LoadImages(string(argv[5]), vstrImageFilenames2, vTimestamps2);
    
    std::cout << "ok3" << std::endl;

    int nImages = vstrImageFilenames.size();
    
    int nImages2 = vstrImageFilenames2.size();
    
    int start1 = 0, start2= 0;

    std::cout << "ok4" << std::endl;
    
    //Start time
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    bool bUseViewer = true;
    ORB_SLAM3::MultiAgentSystem mas(argv[1], true, bUseViewer);
    std::string strSettingsFile1(argv[2]);
    std::string strSettingsFile2(argv[4]);
    std::cout << strSettingsFile1 << std::endl;
    std::cout << strSettingsFile2 << std::endl;
    mas.addAgent(strSettingsFile1);
    mas.addAgent(strSettingsFile2);

    std::atomic<bool> done1(false);
    std::atomic<bool> done2(false);

    std::cout << "ok5" << std::endl;
    
    //Run the thread (Robot1)
    //thread Run(RunSLAM, ref(start1), ref(end1), ref(SLAM), ref(vstrImgFN1), ref(vTS1));
    
    thread Run(RunSLAM, ref(start1), ref(nImages), mas.getAgent(0), ref(vstrImageFilenames), ref(vTimestamps), ref(done1)); 
    
    //Run the thread (Robot2)
//    thread Run2(RunSLAM2, ref(start2), ref(end2), ref(SLAM2), ref(vstrImgFN2), ref(vTS2));
    
    thread Run2(RunSLAM2, ref(start2), ref(nImages2), mas.getAgent(1), ref(vstrImageFilenames2), ref(vTimestamps2), ref(done2));

    
    cout << endl << "-------" << endl;
    cout << "Start processing sequences ..." << endl;
    cout << "Images in the both sequences: " << nImages + nImages2 << endl << endl; 
    
    
    Run.join();
    Run2.join();
    
    while (!done1 || !done2)
    {
        sleep(3);
    }

    mas.Shutdown();
    mas.SaveKFTrajectory();

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }

    std::cout << "there are " << vstrImageFilenames.size() << " images" << std::endl;
}

void RunSLAM(int& start, int& nImages, ORB_SLAM3::Agent* agent, vector<string>& vstrImageFilenames, vector<double>& vTimestamps, std::atomic<bool>& done)
{
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    
    // Main loop
    cv::Mat im;
    for(int ni=start; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        agent->TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }
    done = true;
}

void RunSLAM2(int& start, int& nImages, ORB_SLAM3::Agent* agent, vector<string>& vstrImageFilenames, vector<double>& vTimestamps, std::atomic<bool>& done)
{
        // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    
    // Main loop
    cv::Mat im;
    for(int ni=start; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        agent->TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    done = true;
}
