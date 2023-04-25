#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace robot_detection {

    class SpinEKF
    {
    public:
        Eigen::Matrix<double, 2, 1> x_k1; // k-1时刻的滤波值，即是k-1时刻的值
        Eigen::Matrix<double, 2, 1> K;    // Kalman增益
        Eigen::Matrix<double, 2, 2> F;    // 转移矩阵
        Eigen::Matrix<double, 1, 2> H;    // 观测矩阵
        Eigen::Matrix<double, 2, 2> Q;    // 预测过程噪声偏差的方差
        Eigen::Matrix<double, 1, 1> R;    // 测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
        Eigen::Matrix<double, 2, 2> P;    // 估计误差协方差

        std::string self_type = "SpinEKF";

        // Priori error estimate covariance matrix
        Eigen::Matrix<double, 2, 2> P_pre;
        // Posteriori error estimate covariance matrix
        Eigen::Matrix<double, 2, 2> P_post;

        // Predicted state
        Eigen::VectorXd x_pre;
        // Updated state
        Eigen::Matrix<double, 2, 1> x_post;

        SpinEKF();
        void setF(double t);
        Eigen::Matrix<double, 2, 1> predict();
        Eigen::Matrix<double, 2, 1> update(Eigen::Vector3d z_k);
    };
}