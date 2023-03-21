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
        cv::Rect2f rect;
        Eigen::Vector3d imu_position;
        std::vector<cv::Point2f> points_4;
        std::vector<Eigen::Vector3d> vec_points;
        R_center() = default;
    };

    // 未击打的括号
    struct Buff_no
    {
        cv::RotatedRect in_rrt;
        cv::RotatedRect out_rrt;
        std::vector<cv::Point2d> points_5;
        Eigen::Vector3d imu_position;
        // std::vector<cv::Point2f> contour;
        Buff_no() = default;
    };

    // 击打的括号
    struct Buff_yes
    {
        int number;
        cv::RotatedRect in_rrt;
        cv::RotatedRect out_rrt;
        std::vector<cv::Point2d> points_5;
        Eigen::Vector3d imu_position;
        Buff_yes() = default;
    };


    class BuffDetector
    {
    public:
        BuffDetector();
        void reset();
        bool detectRsult(const cv::Mat src);

        // 0：R未找到，1：找一对括号，2：已经击中一个，5：差最后一个
        int state;
        bool isSmallBuff;
        bool isClockwise;

        AngleSolve AS;
        bool isInitYaw;
        bool initYaw();

        // image
        cv::Mat _src;
        cv::Mat _binary;
        int binary_threshold;
        int enemy_color;
        std::vector<std::vector<cv::Point2f>> all_contours;
	    std::vector<cv::Vec4i> all_hierarchy;
        std::vector<std::vector<cv::Point2f>> r_contours;
        std::vector<cv::Vec4i> r_hierarchy;
        std::vector<std::vector<cv::Point2f>> buff_contours;
        std::vector<cv::Vec4i> buff_hierarchy;
        void setImage(const cv::Mat src);
        void extractContours();
        bool matchColor(std::vector<cv::Point2f> contour);

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

        // Buff_components
        std::vector<cv::RotatedRect> components_rrt;
        bool findComponents();
        bool matchComponents();
        void redefineRotatedRectPoints(cv::Point2f p[], cv::RotatedRect rrt);
        bool isRotateClockwise();

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









    };

}

