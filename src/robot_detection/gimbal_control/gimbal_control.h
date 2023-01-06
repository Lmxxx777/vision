#include "robot_status.h"
#include "opencv2/opencv.hpp"
#include <Eigen/Dense>
#include "armor_detection.hpp"

namespace robot_detection{

    class AngleSolve : public robot_state
    {
    public:
        cv::Mat _src;

        AngleSolve();

        void init(float r, float p, float y, float speed);

        void getAngle1(Armor &aimArmor);

        void getAngle2(Armor aimArmor);

        void getAngle(Eigen::Vector3d predicted_position);

//private:

        double big_w;
        double big_h;
        double small_w;
        double small_h;
        double buff_w;
        double buff_h;

        double fly_time;

        Eigen::Matrix<double,3,3> RotationMatrix_cam2imu;
        Eigen::Matrix<double,3,3> RotationMatrix_imu;

        cv::Mat F_MAT;
        cv::Mat C_MAT;
        Eigen::Matrix<double,3,3> F_EGN;
        Eigen::Matrix<double,1,5> C_EGN;
        Eigen::Matrix<double,3,3> rotated_matrix;
        Eigen::Matrix<double,3,3> coordinate_matrix;

        Eigen::Vector3d cam2imu(Eigen::Vector3d cam_pos);
        Eigen::Vector3d imu2cam(Eigen::Vector3d imu_pos);
        cv::Point2f cam2pixel(Eigen::Vector3d imu_pos);
        cv::Point2f imu2pixel(Eigen::Vector3d imu_pos);
        Eigen::Vector3d pixel2imu(Armor armor, int method);
        Eigen::Vector3d pixel2cam(Armor armor, int method);

        Eigen::Vector3d transformPos2_World(Eigen::Vector3d &Pos);

        Eigen::Vector3d transformPos2_Camera(Eigen::Vector3d &Pos);

        Eigen::Vector3d pnpSolve(cv::Point2f *p, int type, int method);

        Eigen::Vector3d gravitySolve(Eigen::Vector3d &Pos);//just consider gravity no air resistance consider

        Eigen::Vector3d airResistanceSolve(Eigen::Vector3d &Pos);//consider gravity asn air resistance

        Eigen::Vector3d yawPitchSolve(Eigen::Vector3d &Pos);

        float BulletModel(float x, float v, float angle);

        double getFlyTime();
        double getFlyTime(Eigen::Vector3d &pos);
    };

}
