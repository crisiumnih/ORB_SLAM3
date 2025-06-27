
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include<opencv2/videoio.hpp>

#include<System.h>

using namespace std;

int main(int argc, char **argv)
{
    if(argc < 3 || argc > 4)
    {
        cerr << endl << "Usage: ./mono_usb path_to_vocabulary path_to_settings [camera_index]" << endl;
        return 1;
    }

    int camera_index = 0;
    if (argc == 4)
    {
        camera_index = atoi(argv[3]);
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);
    float imageScale = SLAM.GetImageScale();

    //---
    cout << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    //---
    cv::VideoCapture cap(camera_index, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        cerr << "ERROR: Could not open camera" << endl;
        return 1;
    }

    cv::Mat im;
    while (true)
    {
        cap >> im;
        if (im.empty())
        {
            cerr << "ERROR: Failed to capture frame" << endl;
            break;
        }

        cout << "Processing frame..." << endl;

        if(imageScale != 1.f)
        {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,t1.time_since_epoch().count()/1e9);

        if (SLAM.GetTrackingState() == 3) // 3 means LOST
        {
            cout << "WARNING: camera lost!" << endl;
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
