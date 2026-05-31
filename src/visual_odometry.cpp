#include "visual_odometry.hpp"
#include <iostream>

VisualOdometry::VisualOdometry(const cv::Mat& P_left, const cv::Mat& P_right)
    : P_left_(P_left), P_right_(P_right), is_initialized_(false) {
    current_pose_ = cv::Mat::eye(4, 4, CV_64F); 
}

void VisualOdometry::processFrame(const cv::Mat& left_img, const cv::Mat& right_img) {
    if (!is_initialized_) {
        tracker_.extractFeatures(left_img, prev_features_);
        
        // RIGOR: Never trust the sensor. If blind, safely abort this frame.
        if (prev_features_.empty()) {
            std::cerr << "[WARNING] Sensor blind spot. 0 features detected. Waiting..." << std::endl;
            return; 
        }

        std::vector<cv::Point2f> right_features;
        std::vector<uchar> status;
        tracker_.trackFeatures(left_img, right_img, prev_features_, right_features, status);
        
        StereoTriangulation::triangulate(prev_features_, right_features, P_left_, P_right_, prev_points_3d_);
        prev_left_img_ = left_img.clone();
        is_initialized_ = true;
        return;
    }

    // RIGOR: Double check we didn't lose track in the previous frame
    if (prev_features_.empty()) {
        tracker_.extractFeatures(left_img, prev_features_);
        prev_left_img_ = left_img.clone();
        return;
    }

    std::vector<cv::Point2f> curr_features;
    std::vector<uchar> status;
    tracker_.trackFeatures(prev_left_img_, left_img, prev_features_, curr_features, status);

    cv::Mat R, t;
    cv::Mat K = P_left_(cv::Rect(0, 0, 3, 3)); 
    MotionEstimator::estimateMotion(curr_features, prev_points_3d_, K, R, t);

    if (!R.empty() && !t.empty()) {
        cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(T(cv::Rect(0, 0, 3, 3)));
        t.copyTo(T(cv::Rect(3, 0, 1, 3)));
        current_pose_ = current_pose_ * T.inv();
    }

    prev_left_img_ = left_img.clone();
    tracker_.extractFeatures(left_img, prev_features_);
}
