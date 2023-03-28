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
        cv::Point2f pixel_position;
        cv::Point2f  points_4[4];
        cv::RotatedRect rotatedrect;
        cv::Rect rect;
        double radius;
        double distance;
        Eigen::Vector3d buff_position;
        Eigen::Vector3d imu_position;
        Eigen::Vector3d cam_position;
        std::vector<Eigen::Vector3d> points_3d;
        std::vector<cv::Point2f> points_2d;
        R_center() = default;
    };

    // 未击打的括号
    struct Buff_no
    {
        cv::RotatedRect in_rrt;
        cv::RotatedRect out_rrt;
        Eigen::Vector3d buff_position;
        Eigen::Vector3d imu_position;
        Eigen::Vector3d cam_position;
        cv::Point2f pixel_position;
        cv::Point2f points_5[5];
        Buff_no() = default;
    };

    // 击打的括号
    struct Buff_yes
    {
        int number;
        cv::RotatedRect in_rrt;
        cv::RotatedRect out_rrt;
        cv::Point2f points_5[5];
        Eigen::Vector3d imu_position;
        Buff_yes() = default;
    };


    class BuffDetector
    {
    public:
        BuffDetector();
        void reset();
        bool detectResult(const cv::Mat src, chrono_time now_time);

        // 0：R未找到，1：找一对括号，2：已经击中一个，5：差最后一个
        int state;
        double scale_ratio;     // 检测到的坐标和实际坐标的比例缩放
        bool isSmallBuff;
        int buff_type;
        bool isClockwise;
        int rotate_direction;   // 1表示顺时针，-1表示逆时针
        double rotate_speed;
        bool isChange;

        AngleSolve AS;
        bool isInitYaw;
        bool initYaw();

        // image
        cv::Mat _src;
        cv::Mat _binary;
        int binary_threshold;
        int buff_color;
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
        R_center r_center;      
        int fit_circle_counts;
        double r_actual_distance;       // 场地测量的实际距离 
        double error_range;             // 真实的和测量的误差 ±
        double r_max_area;
        double r_min_area;
        double r_full_ratio_min;
        double r_full_ratio_max;
        bool findRcenter();
        bool fitCircle();
        bool calculateScaleRatio();

        // Buff_components
        std::vector<cv::RotatedRect> components_rrt;
        void redefineRotatedRectPoints(cv::Point2f p[], cv::RotatedRect rrt);
        bool findComponents();
        bool matchComponents();

        //Buff calculation
        bool calculateBuffPosition();
        double last_angle;
        Eigen::Vector3d last_vector;
        chrono_time last_time;
        bool isFirstCalculate;
        bool calculateRotateDirectionAndSpeed(chrono_time now_time);

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

