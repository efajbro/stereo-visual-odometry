#include "motion_estimator.hpp"

void MotionEstimator::estimateMotion(const std::vector<cv::Point2f>& points_2d,
                                     const std::vector<cv::Point3f>& points_3d,
                                     const cv::Mat& camera_matrix,
                                     cv::Mat& rotation,
                                     cv::Mat& translation) {
    // We need at least 4 points to physically solve the 3D puzzle. If we have less, abort.
    if (points_2d.size() < 4 || points_3d.size() < 4) return; 

    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F); // Assuming perfectly flat lenses for now
    cv::Mat rvec; // The raw rotation vector

    // Solve the PnP puzzle using RANSAC to ignore the liars (outliers)
    cv::solvePnPRansac(points_3d, points_2d, camera_matrix, dist_coeffs, 
                       rvec, translation, false, 100, 8.0, 0.99);

    // Convert the raw rotation vector into a standard 3x3 rotation matrix for our trajectory map
    cv::Rodrigues(rvec, rotation);
}
