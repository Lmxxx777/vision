#include "robot_status.h"
#include "opencv2/opencv.hpp"
#include <Eigen/Dense>
#include "armor_detection.hpp"

namespace robot_detection{

    class AngleSolve : public robot_state
    {
    public:
        AngleSolve();

        void init(float r, float p, float y, float quat[4], float speed);

        Eigen::Vector3d getAngle(Eigen::Vector3d predicted_position);

        double big_w;
        double big_h;
        double small_w;
        double small_h;
        double buff_r_w;
        double buff_r_h;
        double buff_in_w;
        double buff_in_h;
        double buff_out_w;
        double buff_out_h;
        double buff_radius;
        double buff_convex;

        double fly_time;

        std::string self_type;

        Eigen::Matrix<double,3,3> RotationMatrix_imu; 
        Eigen::Matrix<double,3,3> RotationMatrix_imu2buff; 
        Eigen::Vector3d center_offset_position;
        Eigen::Vector3d gimbal_offset_angle;

        cv::Mat F_MAT;
        cv::Mat C_MAT;
        Eigen::Matrix<double,3,3> F_EGN;
        Eigen::Matrix<double,1,5> C_EGN;
        Eigen::Matrix<double,3,3> rotated_matrix;
        Eigen::Matrix<double,3,3> coordinate_matrix;

        Eigen::Vector3d pnpSolve(std::vector<cv::Point2f> pu, int type);
        Eigen::Vector3d cam2imu(Eigen::Vector3d cam_pos);
        Eigen::Vector3d imu2cam(Eigen::Vector3d imu_pos);
        cv::Point2f cam2pixel(Eigen::Vector3d imu_pos);
        cv::Point2f imu2pixel(Eigen::Vector3d imu_pos);
        Eigen::Vector3d pixel2cam(Armor &armor, int type);
        Eigen::Vector3d pixel2imu(Armor &armor, int type);
        // for BUFF
        Eigen::Vector3d pixel2cam(std::vector<cv::Point2f> points, int type);
        Eigen::Vector3d pixel2imu(std::vector<cv::Point2f> points, int type);
        Eigen::Vector3d imu2buff(Eigen::Vector3d imu_pos);
        Eigen::Vector3d buff2imu(Eigen::Vector3d buff_pos);

        Eigen::Vector3d airResistanceSolve(Eigen::Vector3d Pos);//consider gravity asn air resistance
        float BulletModel(float x, float v, float angle);
        Eigen::Vector3d yawPitchSolve(Eigen::Vector3d &Pos);
        double getFlyTime(Eigen::Vector3d &pos);

        Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta);
        Eigen::Matrix3d eulerAnglesToRotationMatrix2(Eigen::Vector3d &theta);
        Eigen::Matrix3d quaternionToRotationMatrix(float quaternion[4]);
        bool pointsInLine(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, double error, int type);  // 三点是否在误差允许内为一条直线，type的值为1表示斜率，0表示夹角
        bool circleLeastFit(const std::vector<cv::Point2f> &points, double &center_x, double &center_y, double &radius);  // 对二维平面点集拟合成圆
        double countArmorIoU(Armor armor1, Armor armor2);   // 输出像素坐标系内两旋转矩形的交并比
    };

}