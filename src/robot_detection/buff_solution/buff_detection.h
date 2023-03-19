#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>



namespace robot_detection
{
    class BuffDetector
    {
    public:

        void setImage(cv::Mat src)
        {
            
        }
        
        // R标
        cv::Rect2f R_Rect;
        cv::Point2f R_center;
        Eigen::Vector3d R_center_imu;

        bool findRcenter();

        // 未击打的括号
        cv::RotatedRect in_rrt;
        cv::RotatedRect out_rrt;
        std::vector<cv::Point2d> ing_pixel_pts;




    };

}

