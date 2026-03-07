#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <System.h>
#include <ImuTypes.h>

using namespace std;

int main(int argc, char **argv) {
    if (argc != 4) {
        cerr << "Usage: ./rgbd_inertial_streaming <vocabulary> <settings> <trajectory_output>" << endl;
        return 1;
    }

    string vocabulary_path = argv[1];
    string settings_path = argv[2];
    string trajectory_path = argv[3];

    ORB_SLAM3::System SLAM(vocabulary_path, settings_path, ORB_SLAM3::System::IMU_RGBD, false);
    float imageScale = SLAM.GetImageScale();

    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    int frame_count = 0;

    string line;
    while (getline(cin, line)) {
        if (line.empty())
            continue;

        istringstream iss(line);
        string command;
        iss >> command;

        if (command == "IMU") {
            double ts;
            float ax, ay, az, gx, gy, gz;
            if (!(iss >> ts >> ax >> ay >> az >> gx >> gy >> gz)) {
                cerr << "Invalid IMU line: " << line << endl;
                continue;
            }
            vImuMeas.emplace_back(ax, ay, az, gx, gy, gz, ts);
        } else if (command == "FRAME") {
            double ts;
            string rgb_path, depth_path;
            if (!(iss >> ts >> rgb_path >> depth_path)) {
                cerr << "Invalid FRAME line: " << line << endl;
                continue;
            }

            cv::Mat im = cv::imread(rgb_path, cv::IMREAD_COLOR);
            cv::Mat depth = cv::imread(depth_path, cv::IMREAD_UNCHANGED);

            if (im.empty()) {
                cerr << "Failed to load RGB: " << rgb_path << endl;
                continue;
            }
            if (depth.empty()) {
                cerr << "Failed to load depth: " << depth_path << endl;
                continue;
            }

            if (imageScale != 1.f) {
                int width = im.cols * imageScale;
                int height = im.rows * imageScale;
                cv::resize(im, im, cv::Size(width, height));
                cv::resize(depth, depth, cv::Size(width, height));
            }

            SLAM.TrackRGBD(im, depth, ts, vImuMeas);
            vImuMeas.clear();
            frame_count++;

            if (frame_count % 100 == 0)
                cerr << "Processed " << frame_count << " frames" << endl;
        } else if (command == "END") {
            break;
        } else {
            cerr << "Unknown command: " << command << endl;
        }
    }

    cerr << "Shutting down after " << frame_count << " frames" << endl;
    SLAM.Shutdown();
    SLAM.SaveTrajectoryTUM(trajectory_path);

    cout << "DONE" << endl;
    cout.flush();

    return 0;
}
