/**
* Modified version to process RealSense .bag files (RGB-D only)
* Usage: ./rgbd_realsense_bag path_to_vocabulary path_to_settings path_to_bag_file
*/

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"

#include <System.h>

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);

int main(int argc, char **argv) {

    if (argc != 4) {
        cerr << endl
             << "Usage: ./rgbd_realsense_bag path_to_vocabulary path_to_settings path_to_bag_file"
             << endl;
        cerr << "Example: ./rgbd_realsense_bag Vocabulary/ORBvoc.txt settings.yaml recording.bag" << endl;
        return 1;
    }

    string bag_file = string(argv[3]);
    
    // Check if bag file exists
    ifstream file_check(bag_file);
    if (!file_check.good()) {
        cerr << "Error: Bag file not found: " << bag_file << endl;
        return 1;
    }
    file_check.close();

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    // Create pipeline for bag file playback
    rs2::pipeline pipe;
    rs2::config cfg;
    
    // Enable device from bag file
    cfg.enable_device_from_file(bag_file);
    
    cout << "Loading bag file: " << bag_file << endl;
    
    // Start pipeline
    rs2::pipeline_profile pipe_profile;
    try {
        pipe_profile = pipe.start(cfg);
    } catch (const rs2::error & e) {
        cerr << "RealSense error: " << e.what() << endl;
        return 1;
    }
    
    // Get playback device and disable real-time mode for faster processing
    auto device = pipe_profile.get_device();
    auto playback = device.as<rs2::playback>();
    playback.set_real_time(false);
    
    cout << "Playback mode set to non-realtime (faster processing)" << endl;

    // Get stream profile info
    rs2::stream_profile color_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);
    rs2::stream_profile depth_stream = pipe_profile.get_stream(RS2_STREAM_DEPTH);
    
    rs2_intrinsics color_intrinsics = color_stream.as<rs2::video_stream_profile>().get_intrinsics();
    
    cout << "\nCamera Intrinsics:" << endl;
    cout << "  fx = " << color_intrinsics.fx << endl;
    cout << "  fy = " << color_intrinsics.fy << endl;
    cout << "  cx = " << color_intrinsics.ppx << endl;
    cout << "  cy = " << color_intrinsics.ppy << endl;
    cout << "  width = " << color_intrinsics.width << endl;
    cout << "  height = " << color_intrinsics.height << endl;
    cout << "  Distortion coeffs: [" << color_intrinsics.coeffs[0] << ", " 
         << color_intrinsics.coeffs[1] << ", " << color_intrinsics.coeffs[2] << ", " 
         << color_intrinsics.coeffs[3] << ", " << color_intrinsics.coeffs[4] << "]" << endl;

    // Align depth to color
    rs2_stream align_to = find_stream_to_align(pipe_profile.get_streams());
    rs2::align align(align_to);

    // Create SLAM system (no viewer for macOS compatibility)
    cout << "\nInitializing ORB-SLAM3..." << endl;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, false);
    float imageScale = SLAM.GetImageScale();
    
    cout << "Image scale: " << imageScale << endl;
    cout << "\n========================================" << endl;
    cout << "Processing bag file..." << endl;
    cout << "Press Ctrl+C to stop" << endl;
    cout << "========================================\n" << endl;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    int frame_count = 0;
    const int MAX_FRAMES = 30;  // Limit for testing
    
    cout << "Processing up to " << MAX_FRAMES << " frames..." << endl;
    
    try {
        while (b_continue_session && frame_count < MAX_FRAMES) {
            // Wait for frames
            rs2::frameset frames;
            if (!pipe.try_wait_for_frames(&frames, 5000)) {
                // End of bag file or timeout
                cout << "End of bag file reached or timeout" << endl;
                break;
            }

            // Align depth to color
            auto aligned_frames = align.process(frames);
            
            // Get color and depth frames
            rs2::video_frame color_frame = aligned_frames.get_color_frame();
            rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();
            
            if (!color_frame || !depth_frame) {
                continue;
            }

            // Get timestamp (in seconds)
            double timestamp = frames.get_timestamp() / 1000.0;

            // Convert to OpenCV Mat
            cv::Mat im(cv::Size(color_intrinsics.width, color_intrinsics.height), 
                      CV_8UC3, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);
            cv::Mat depth(cv::Size(color_intrinsics.width, color_intrinsics.height), 
                         CV_16U, (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP);

            // Resize if needed
            if(imageScale != 1.f) {
                int width = im.cols * imageScale;
                int height = im.rows * imageScale;
                cv::resize(im, im, cv::Size(width, height));
                cv::resize(depth, depth, cv::Size(width, height));
            }

            // Track with ORB-SLAM3
            auto t1 = std::chrono::steady_clock::now();
            SLAM.TrackRGBD(im, depth, timestamp);
            auto t2 = std::chrono::steady_clock::now();
            
            double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            vTimesTrack.push_back(ttrack);
            
            frame_count++;
            
            // Print progress every 30 frames
            if (frame_count % 30 == 0) {
                cout << "Processed " << frame_count << " frames | "
                     << "Tracking time: " << ttrack*1000.0 << " ms" << endl;
            }
        }
    } catch (const rs2::error & e) {
        cerr << "RealSense error during processing: " << e.what() << endl;
    }

    // Print statistics
    cout << "\n========================================" << endl;
    cout << "Processing complete!" << endl;
    cout << "========================================" << endl;
    cout << "Total frames processed: " << frame_count << endl;
    
    if (vTimesTrack.size() > 0) {
        sort(vTimesTrack.begin(), vTimesTrack.end());
        float totaltime = 0;
        for(size_t i=0; i<vTimesTrack.size(); i++) {
            totaltime += vTimesTrack[i];
        }
        cout << "Median tracking time: " << vTimesTrack[vTimesTrack.size()/2] << " s" << endl;
        cout << "Mean tracking time: " << totaltime/vTimesTrack.size() << " s" << endl;
    }

    // Stop pipeline first
    cout << "\nStopping pipeline..." << endl;
    try {
        pipe.stop();
        cout << "Pipeline stopped successfully" << endl;
    } catch (const rs2::error & e) {
        cerr << "Warning: Error stopping pipeline: " << e.what() << endl;
    }
    
    // Save trajectory BEFORE shutdown
    cout << "\nSaving trajectories..." << endl;
    try {
        SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
        cout << "  - CameraTrajectory.txt saved" << endl;
    } catch (const std::exception& e) {
        cerr << "Error saving camera trajectory: " << e.what() << endl;
    }
    
    try {
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        cout << "  - KeyFrameTrajectory.txt saved" << endl;
    } catch (const std::exception& e) {
        cerr << "Error saving keyframe trajectory: " << e.what() << endl;
    }
    
    // Shutdown SLAM (this is where the crash might occur)
    cout << "\nShutting down SLAM system..." << endl;
    try {
        SLAM.Shutdown();
        cout << "SLAM shutdown completed" << endl;
    } catch (const std::exception& e) {
        cerr << "Error during SLAM shutdown: " << e.what() << endl;
    } catch (...) {
        cerr << "Unknown error during SLAM shutdown" << endl;
    }
    
    cout << "\n✅ All operations complete!" << endl;

    return 0;
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

