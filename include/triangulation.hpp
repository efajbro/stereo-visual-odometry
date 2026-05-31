#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

class StereoTriangulation {
public:
    // Takes 2D pixels from Left and Right cameras and calculates their physical 3D coordinates
    static void triangulate(const std::vector<cv::Point2f>& pts_left,
                            const std::vector<cv::Point2f>& pts_right,
                            const cv::Mat& P_left,
                            const cv::Mat& P_right,
                            std::vector<cv::Point3f>& points_3d);
};
