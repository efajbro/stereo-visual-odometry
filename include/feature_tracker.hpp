#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class FeatureTracker {
public:
    // Constructor: Sets up the spatial bucketing grid
    FeatureTracker(int grid_rows = 4, int grid_cols = 4, int max_features_per_bucket = 20);

    // Extracts evenly distributed corners across the image
    void extractFeatures(const cv::Mat& image, std::vector<cv::Point2f>& features);

    // Uses KLT Optical Flow to track those corners into the next frame
    void trackFeatures(const cv::Mat& prev_image, const cv::Mat& curr_image, 
                       std::vector<cv::Point2f>& prev_features, 
                       std::vector<cv::Point2f>& curr_features, 
                       std::vector<uchar>& status);

private:
    int grid_rows_;
    int grid_cols_;
    int max_features_per_bucket_;

    void applySpatialBucketing(const std::vector<cv::KeyPoint>& raw_keypoints, 
                               std::vector<cv::Point2f>& filtered_features, 
                               int img_width, int img_height);
};
