#include <Eigen/Dense>

class KalmanFilter {

public:
    Eigen::VectorXd x_k1; // k-1时刻的滤波值，即是k-1时刻的值
    Eigen::MatrixXd K;    // Kalman增益
    Eigen::MatrixXd F;    // 转移矩阵
    Eigen::MatrixXd H;    // 观测矩阵
    Eigen::MatrixXd Q;    // 预测过程噪声偏差的方差
    Eigen::MatrixXd R;    // 测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
    Eigen::MatrixXd P;    // 估计误差协方差

    KalmanFilter() = default;

    void initial(Eigen::Vector3d position);

    void initial(Eigen::Vector3d position, Eigen::Vector3d speed);

    void setF(double t);

    void setP(Eigen::MatrixXd P_last);

    Eigen::VectorXd update(Eigen::Vector3d z_k);

};
