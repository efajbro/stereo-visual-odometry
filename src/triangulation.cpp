#include "triangulation.hpp"

void StereoTriangulation::triangulate(const std::vector<cv::Point2f>& pts_left,
                                      const std::vector<cv::Point2f>& pts_right,
                                      const cv::Mat& P_left,
                                      const cv::Mat& P_right,
                                      std::vector<cv::Point3f>& points_3d) {
    if (pts_left.empty() || pts_right.empty()) return;

    cv::Mat points_4d;
    // OpenCV's high-speed linear triangulation matrix algebra
    cv::triangulatePoints(P_left, P_right, pts_left, pts_right, points_4d);

    points_3d.clear();
    // Convert the raw mathematical 4D matrix output into standard 3D (X, Y, Z) coordinates
    for (int i = 0; i < points_4d.cols; i++) {
        float w = points_4d.at<float>(3, i);
        float x = points_4d.at<float>(0, i) / w;
        float y = points_4d.at<float>(1, i) / w;
        float z = points_4d.at<float>(2, i) / w;
        points_3d.push_back(cv::Point3f(x, y, z));
    }
}
