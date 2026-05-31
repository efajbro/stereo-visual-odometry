#include "feature_tracker.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

int main() {
    // Read from the real-world driving video we just downloaded
    cv::VideoCapture cap("../tests/driving.mp4");
    if (!cap.isOpened()) {
        std::cerr << "[ERROR] Could not open the video file." << std::endl;
        return -1;
    }

    // Boot your Feature Tracker (4x4 spatial grid, max 20 features per grid)
    FeatureTracker tracker(4, 4, 20);
    
    cv::Mat old_frame, old_gray;
    std::vector<cv::Point2f> p0, p1;

    // Grab the first frame to initialize the tracker
    cap.read(old_frame);
    // Resize for speed and consistency
    cv::resize(old_frame, old_frame, cv::Size(640, 480));
    cv::cvtColor(old_frame, old_gray, cv::COLOR_BGR2GRAY);
    tracker.extractFeatures(old_gray, p0);

    cv::Mat mask = cv::Mat::zeros(old_frame.size(), old_frame.type());
    std::cout << "[INFO] Real-World Video Tracker Booted. Press 'ESC' to exit." << std::endl;

    while (true) {
        cv::Mat frame, frame_gray;
        cap.read(frame);
        if (frame.empty()) {
            std::cout << "[INFO] Video ended." << std::endl;
            break;
        }
        
        cv::resize(frame, frame, cv::Size(640, 480));
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        // Run YOUR KLT Optical Flow engine on the real-life feed
        std::vector<uchar> status;
        tracker.trackFeatures(old_gray, frame_gray, p0, p1, status);

        std::vector<cv::Point2f> good_new;
        for (uint i = 0; i < p0.size(); i++) {
            if (status[i] == 1) {
                good_new.push_back(p1[i]);
                // Draw a green tracking line (optical flow vector)
                cv::line(mask, p1[i], p0[i], cv::Scalar(0, 255, 0), 2);
                // Draw a red dot at the tracked feature
                cv::circle(frame, p1[i], 4, cv::Scalar(0, 0, 255), -1);
            }
        }

        cv::Mat img;
        cv::add(frame, mask, img);
        
        cv::imshow("REAL LIFE: FAST + KLT Sub-Pixel Tracking", img);

        old_gray = frame_gray.clone();
        p0 = good_new;

        // If points drop off screen, re-detect fresh corners
        if (p0.size() < 40) {
            tracker.extractFeatures(old_gray, p0);
            mask = cv::Mat::zeros(old_frame.size(), old_frame.type()); 
        }

        // Wait 30ms so it plays at normal speed
        if (cv::waitKey(30) == 27) break; 
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
