#pragma once
#include <opencv2/opencv.hpp>
#include "feature_tracker.hpp"
#include "triangulation.hpp"
#include "motion_estimator.hpp"

class VisualOdometry {
public:
    // Initialize the engine with the factory calibration matrices of the Left and Right cameras
    VisualOdometry(const cv::Mat& P_left, const cv::Mat& P_right);

    // The main engine loop. Feed it the live video frames, and it calculates the car's movement.
    void processFrame(const cv::Mat& left_img, const cv::Mat& right_img);

    // Outputs the current physical position of the car in the 3D world
    cv::Mat getCurrentPose() const { return current_pose_; }

private:
    cv::Mat P_left_, P_right_;
    cv::Mat current_pose_; // A 4x4 matrix storing the car's total X, Y, Z position and Rotation
    
    FeatureTracker tracker_;
    cv::Mat prev_left_img_;
    std::vector<cv::Point2f> prev_features_;
    std::vector<cv::Point3f> prev_points_3d_;
    bool is_initialized_;
};
