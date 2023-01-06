#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#define PI CV_PI

namespace robot_detection {
    //二维Singer模型
    class Skalman
    {
        double alefa = 1.0/20.0;//目标机动频率
        double Sigmaq = 0.01;//目标加速度标准差，Singer模型假设目标加速度符合均值为零的高斯分布
        double initT = 0.03;//用来初始化初始协方差矩阵的采样时间间隔（估算出来的）
        double lamda;//渐消因子，减小滤波发散问题
        Eigen::Matrix<double, 6, 6> F;//状态转移矩阵
        Eigen::Matrix<double, 6, 6> W;//预测方程过程噪声
        Eigen::Matrix<double, 6, 1> Xk_1;//转移的状态
    //    Eigen::Matrix<double, 6, 1> Xk;
        Eigen::Matrix<double, 2, 1> Zk;//观测值
        Eigen::Matrix<double, 2, 6> H;//观测矩阵
        Eigen::Matrix<double, 6, 2> K;//卡尔曼增益
        Eigen::Matrix<double, 6, 6> P;//协方差矩阵
        Eigen::Matrix<double, 2, 2> R;//测量过程噪声
        Eigen::Matrix<double, 6, 2> G;//加速度控制矩阵
        Eigen::Matrix<double, 2, 1> _a;//平均加速度，传递加速度大致范围
        Eigen::Matrix<double, 2, 1> Vk;//新息序列，计算渐消因子需要
        Eigen::Matrix<double, 2, 2> _Sk;//根据测量值算出来的新息协方差
        Eigen::Matrix<double, 2, 2> Sk;//根据观测方程算出来的新息协方差
    public:
        double T = 0;//采样周期T，即前后两次预测帧相隔的时间

        Skalman();
        void Reset();
        void Reset(Eigen::Vector3d &Xpos);
        void PredictInit(double deleta_t);
        void setXpos(Eigen::Vector3d &Xpos);
        Eigen::Matrix<double,6,1> predict(bool predict)
        {
            _a << Xk_1(2,0), Xk_1(5,0);
            if(predict)
            {
                return F*Xk_1 + G*_a;
            }
            Xk_1 = F*Xk_1 + G*_a;
    //        std::cout<<"predictData:"<<Xk_1<<std::endl;
            return Xk_1;
        }

        Eigen::Matrix<double,6,1> correct(Eigen::Matrix<double,2,1> &measure)
        {
            Zk = measure;
            Vk = Zk - H*Xk_1;
            _Sk = (lamda*Vk*Vk.transpose())/(1+lamda);
            Sk = H*P*H.transpose() + R;
            lamda = std::max(1.,_Sk.trace()/Sk.trace());
    //        std::cout<<"lamda:"<<lamda<<std::endl;
            P = lamda*((F * P * F.transpose()) + W);
            K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
            Xk_1 = Xk_1 + K * (Zk - H * Xk_1);
            P = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P;
    //        std::cout<<"correct_Data:"<<Xk_1<<"-----------------------------"<<std::endl;
            return Xk_1;
        }

    };
}