#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "robot_status.h"
#include "gimbal_control.h"


namespace robot_detection
{

    // R标
    struct R_center
    {
        cv::Rect2f r_rect;
        Eigen::Vector3d r_center_imu_position;
        std::vector<cv::Point2f> r_rect_points;
        std::vector<Eigen::Vector3d> r_center_points;
        R_center() = default;
    };

    // 未击打的括号
    struct Buff_no
    {
        cv::RotatedRect in_rrt;
        cv::RotatedRect out_rrt;
        std::vector<cv::Point2d> points;
        // std::vector<cv::Point2f> contour;
        Buff_no() = default;
    };

    // 击打的括号
    struct Buff_yes
    {
        int number;
        cv::RotatedRect in_rrt;
        cv::RotatedRect out_rrt;
        std::vector<cv::Point2d> points;

        Buff_yes() = default;
    };


    class BuffDetector
    {
    public:
        BuffDetector();
        bool detectRsult(const cv::Mat src);

        // 0：R未找到，1：找一对括号，2：已经击中一个，5：差最后一个
        int state;

        AngleSolve AS;

        // image
        cv::Mat _src;
        cv::Mat _binary;
        int binary_threshold;
        int enemy_color;
        std::vector<std::vector<cv::Point2f>> contours;
	    std::vector<cv::Vec4i> hierarchy;
        void setImage(const cv::Mat src);
        void extractContours();

        // R
        bool isFindR;
        R_center r_center;      // TODO: 是否需要做成vector来拟合圆心
        int fit_circle_counts;
        double r_max_area;
        double r_min_area;
        double r_full_ratio_min;
        double r_full_ratio_max;
        bool findRcenter();
        bool fitCircle();

        // Buff_no
        Buff_no buff_no;
        double no_buff_area_min;
        double no_buff_area_max;
        double no_full_ratio_min;
        double no_full_ratio_max;

        // Buff_yes
        std::vector<Buff_yes> buff_yes;
        double yes_buff_area_max;
        double yes_buff_area_min;
        double yes_full_ratio_min;
        double yes_full_ratio_max;

        // Buff_components
        std::vector<cv::RotatedRect> components_rrt;
        bool findComponents();
        bool matchComponents();








    };

}

