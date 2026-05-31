#include "feature_tracker.hpp"
#include <algorithm>

FeatureTracker::FeatureTracker(int grid_rows, int grid_cols, int max_features_per_bucket)
    : grid_rows_(grid_rows), grid_cols_(grid_cols), max_features_per_bucket_(max_features_per_bucket) {}

void FeatureTracker::applySpatialBucketing(const std::vector<cv::KeyPoint>& raw_keypoints, 
                                           std::vector<cv::Point2f>& filtered_features, 
                                           int img_width, int img_height) {
    int bucket_width = img_width / grid_cols_;
    int bucket_height = img_height / grid_rows_;
    std::vector<std::vector<cv::Point2f>> buckets(grid_rows_ * grid_cols_);

    for (const auto& kp : raw_keypoints) {
        int col = std::min(static_cast<int>(kp.pt.x) / bucket_width, grid_cols_ - 1);
        int row = std::min(static_cast<int>(kp.pt.y) / bucket_height, grid_rows_ - 1);
        buckets[row * grid_cols_ + col].push_back(kp.pt);
    }

    for (auto& bucket : buckets) {
        int count = 0;
        for (const auto& pt : bucket) {
            if (count >= max_features_per_bucket_) break;
            filtered_features.push_back(pt);
            count++;
        }
    }
}

void FeatureTracker::extractFeatures(const cv::Mat& image, std::vector<cv::Point2f>& features) {
    std::vector<cv::KeyPoint> raw_keypoints;
    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(20, true);
    detector->detect(image, raw_keypoints);
    
    features.clear();
    applySpatialBucketing(raw_keypoints, features, image.cols, image.rows);
}

void FeatureTracker::trackFeatures(const cv::Mat& prev_image, const cv::Mat& curr_image, 
                                   std::vector<cv::Point2f>& prev_features, 
                                   std::vector<cv::Point2f>& curr_features, 
                                   std::vector<uchar>& status) {
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_image, curr_image, prev_features, curr_features, status, err);
}
