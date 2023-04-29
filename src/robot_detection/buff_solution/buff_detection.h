#include <iostream>
#include <cmath>
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

        // 检测到的坐标和实际坐标的比例缩放
        double symbol_scale_ratio;      // R的对角线的实际长度比测距长度
        double radius_scale_ratio;      // 未击打符叶靶心和R中心的实际距离比测距距离

        bool isSmallBuff;       // 默认为小符，小符为true，大符为false
        int buff_type;          // 默认为小符，小符为1，大符为0

        bool isClockwise;       // true表示顺时针，false表示逆时针
        int rotate_direction;   // -1表示顺时针，1表示逆时针     ------------!!!!!!---attention---!!!!!!------------
        double current_angle;
        double rotate_speed;
        double const_rotate_speed;  // 60°/s

        Eigen::Vector3d bullet_position;

        AngleSolve AS;
        bool isInitYaw;
        bool initYaw();

        // image
        cv::Mat _src;
        cv::Mat _binary;
        int binary_threshold;
        int buff_color;
        std::vector<std::vector<cv::Point>> all_contours;
	    std::vector<cv::Vec4i> all_hierarchy;
        std::vector<std::vector<cv::Point>> r_contours;
        std::vector<cv::Vec4i> r_hierarchy;
        std::vector<std::vector<cv::Point>> buff_contours;
        std::vector<cv::Vec4i> buff_hierarchy;
        void setImage();
        void extractContours();
        bool matchColor(std::vector<cv::Point> contour);

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
        int r_width_pixel;
        int r_height_pixel; 
        // double r_
        bool findRcenter();
        bool fitCircle();

        // Buff_components
        std::vector<cv::RotatedRect> components_rrt;
        void redefineRotatedRectPoints(cv::Point2f p[], cv::RotatedRect rrt);
        bool findComponents();
        bool matchComponents();

        //Buff calculation
        double maintainAngleLegal(double angle);
        bool calculateBuffPosition();
        bool calculateScaleRatio();
        bool isSwitchBuff();
        double last_angle;
        Eigen::Vector3d last_vector;
        chrono_time last_time;
        bool isFirstCalculate;
        bool calculateRotateDirectionAndSpeed(chrono_time now_time);
        bool calculateShootPosition();

        // buff feature
        // speed = a * sin(w * t) + b
        std::vector<double> speed_vector;
        double a;
        double w;
        double b;
        int fit_sinusoid_counts;    // 314 ~ 335 (>400)
        double fit_sinusoid_time;   // PI/2 ~ 1.667s   (whole T: PI ~ 3.335)
        bool fitSinusoid();
        chrono_time begin_time;
        bool isSwitch;
        bool isBegin;

        // Buff_no
        Buff_no buff_no;        // 当前帧的未击打符叶
        Buff_no buff_no_last;   // 过去帧的未击打符叶
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

