#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

class MotionEstimator {
public:
    // Calculates the physical rotation and translation (movement) of the camera
    static void estimateMotion(const std::vector<cv::Point2f>& points_2d,
                               const std::vector<cv::Point3f>& points_3d,
                               const cv::Mat& camera_matrix,
                               cv::Mat& rotation,
                               cv::Mat& translation);
};
