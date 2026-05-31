#include "visual_odometry.hpp"
#include <iostream>
#include <vector>

int main() {
    cv::Mat P_left = (cv::Mat_<double>(3, 4) << 718.8, 0, 607.1, 0,
                                                0, 718.8, 185.2, 0,
                                                0, 0, 1, 0);
    cv::Mat P_right = (cv::Mat_<double>(3, 4) << 718.8, 0, 607.1, -386.1, 
                                                 0, 718.8, 185.2, 0,
                                                 0, 0, 1, 0);

    VisualOdometry engine(P_left, P_right);
    
    cv::RNG rng(12345); 
    std::vector<cv::Point2f> object_texture;
    for(int j = 0; j < 100; j++) {
        object_texture.push_back(cv::Point2f(rng.uniform(10, 190), rng.uniform(10, 190)));
    }

    // Create a larger, cleaner 700x600 canvas for the trajectory
    cv::Mat trajectory_map = cv::Mat::zeros(700, 600, CV_8UC3);
    
    // Track the previous point to draw continuous lines
    cv::Point prev_pixel(300, 500); 
    char text_buf[100];

    for (int i = 0; i < 100; i++) {
        cv::Mat left_img = cv::Mat::zeros(370, 1226, CV_8UC1);
        cv::Mat right_img = cv::Mat::zeros(370, 1226, CV_8UC1);
        
        int shift = i * 4; 
        int obj_x = 700 - shift;
        int obj_y = 150;
        
        // Draw object geometry
        cv::rectangle(left_img, cv::Point(obj_x, obj_y), cv::Point(obj_x + 200, obj_y + 200), cv::Scalar(60), -1);
        cv::rectangle(right_img, cv::Point(obj_x - 50, obj_y), cv::Point(obj_x - 50 + 200, obj_y + 200), cv::Scalar(60), -1); 

        for(const auto& pt : object_texture) {
            cv::circle(left_img, cv::Point(obj_x + pt.x, obj_y + pt.y), 3, cv::Scalar(255), -1);
            cv::circle(right_img, cv::Point(obj_x - 50 + pt.x, obj_y + pt.y), 3, cv::Scalar(255), -1);
        }

        engine.processFrame(left_img, right_img);
        cv::Mat pose = engine.getCurrentPose();
        
        double x_move = pose.at<double>(0, 3);
        double z_move = pose.at<double>(2, 3);

        // Map real world meters to pixels safely (Scale factor: 5 pixels per meter)
        int draw_x = 300 + static_cast<int>(x_move * 5);
        int draw_y = 500 - static_cast<int>(z_move * 5); 
        cv::Point curr_pixel(draw_x, draw_y);

        // Draw a continuous green tracking line if the math is within bounds
        if (draw_x >= 0 && draw_x < 600 && draw_y >= 0 && draw_y < 700) {
            cv::line(trajectory_map, prev_pixel, curr_pixel, CV_RGB(0, 255, 0), 2);
            cv::circle(trajectory_map, curr_pixel, 3, CV_RGB(255, 0, 0), -1); // Red dot for current head
            prev_pixel = curr_pixel;
        }

        // Wipe out the top 80 pixels exclusively for the text dashboard display area
        cv::rectangle(trajectory_map, cv::Point(0, 0), cv::Point(600, 80), CV_RGB(15, 15, 15), -1);
        cv::line(trajectory_map, cv::Point(0, 80), cv::Point(600, 80), CV_RGB(50, 50, 50), 1);

        // FIX: High-precision float printing string formatting
        std::snprintf(text_buf, sizeof(text_buf), "VO ENGINE | X: %.2fm | Z: %.2fm", x_move, z_move);
        cv::putText(trajectory_map, text_buf, cv::Point(20, 45), cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(255, 255, 255), 2);

        // Render the windows
        cv::imshow("Left Camera Input Stream", left_img);
        cv::imshow("Calculated 2D Trajectory Map", trajectory_map);
        
        cv::waitKey(40); 
    }
    
    cv::waitKey(0);
    return 0;
}
