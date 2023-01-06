#include "singer_prediction.hpp"

namespace robot_detection {

    Skalman::Skalman()
    {

        //init H
        H << 1,0,0,0,0,0,
            0,0,0,1,0,0;

        //init R
        R << 0.005, 0,
            0, 0.005;

        //init Xk_1
        Xk_1 << 0,0.001,0.01,
                0,0.001,0.01;

        //init Xk
    //    Xk = Xk_1;

        //init P
        double r11 = R(0,0);
        double r22 = R(1,1);
        Eigen::Matrix<double,3,3> p11,p22;
        p11 <<r11, r11/initT, r11/(initT*initT),
                r11/initT, (2*r11)/(initT*initT), (3*r11)/pow(initT,3),
                r11/(initT*initT), (3*r11)/pow(initT,3), (4*r11)/pow(initT,4);

        p22<<r22, r22/initT, r22/(initT*initT),
                r22/initT, (2*r22)/(initT*initT), (3*r22)/pow(initT,3),
                r22/(initT*initT), (3*r22)/pow(initT,3), (4*r22)/pow(initT,4);

        P<<p11,Eigen::Matrix<double,3,3>::Zero(),
                Eigen::Matrix<double,3,3>::Zero(),p22;

        //init Zk
        Zk << 0.1,0.1;

        //init lamda
        Vk = Zk - H*Xk_1;
        _Sk = 0.5*Vk*Vk.transpose();
        Sk = H*P*H.transpose() + R;
        lamda = std::max(1.,_Sk.trace()/Sk.trace());
    //    std::cout<<"lamda:"<<lamda<<std::endl;
    }

    void Skalman::Reset()
    {
        //init Xk_1
        Xk_1 << 0,0.001,0.01,
                0,0.001,0.01;

        //init Xk
    //    Xk = Xk_1;

        //init P
        double r11 = R(0,0);
        double r22 = R(1,1);
        Eigen::Matrix<double,3,3> p11,p22;
        p11 <<r11, r11/initT, r11/(initT*initT),
                r11/initT, (2*r11)/(initT*initT), (3*r11)/pow(initT,3),
                r11/(initT*initT), (3*r11)/pow(initT,3), (4*r11)/pow(initT,4);

        p22<<r22, r22/initT, r22/(initT*initT),
                r22/initT, (2*r22)/(initT*initT), (3*r22)/pow(initT,3),
                r22/(initT*initT), (3*r22)/pow(initT,3), (4*r22)/pow(initT,4);

        P<<p11,Eigen::Matrix<double,3,3>::Zero(),
                Eigen::Matrix<double,3,3>::Zero(),p22;

        //init Zk
        Zk << 0.1,0.1;
    }


    void Skalman::Reset(Eigen::Vector3d &Xpos)
    {
        //init Xk_1
        Xk_1 << Xpos(0,0),0.001,0.01,
                Xpos(1,0),0.001,0.01;

        //init Xk
    //    Xk = Xk_1;

        //init P
        double r11 = R(0,0);
        double r22 = R(1,1);
        Eigen::Matrix<double,3,3> p11,p22;
        p11 <<r11, r11/initT, r11/(initT*initT),
                r11/initT, (2*r11)/(initT*initT), (3*r11)/pow(initT,3),
                r11/(initT*initT), (3*r11)/pow(initT,3), (4*r11)/pow(initT,4);

        p22<<r22, r22/initT, r22/(initT*initT),
                r22/initT, (2*r22)/(initT*initT), (3*r22)/pow(initT,3),
                r22/(initT*initT), (3*r22)/pow(initT,3), (4*r22)/pow(initT,4);

        P<<p11,Eigen::Matrix<double,3,3>::Zero(),
                Eigen::Matrix<double,3,3>::Zero(),p22;

        //init Zk
        Zk << 0.1,0.1;
    }

    void Skalman::setXpos(Eigen::Vector3d &Xpos)
    {
        Xk_1 << Xpos(0,0),0.001,0.01,
                Xpos(1,0),0.001,0.01;
    }

    void Skalman::PredictInit(double deleta_t)
    {
        T = deleta_t;
    //    std::cout<<"time_T:"<<(1-exp(-alefa*T))/alefa<<std::endl;

        //init F
        Eigen::Matrix<double,3,3> Fx;
        Eigen::Matrix<double,3,3> Fy;
        Fx<<1, T, (alefa*T-1+exp(-alefa*T))/(alefa*alefa),
                0, 1, (1-exp(-alefa*T))/alefa,
                0, 0, exp(-alefa*T);
        Fy = Fx;

        F << Fx,Eigen::Matrix<double,3,3>::Zero(),
                Eigen::Matrix<double,3,3>::Zero(),Fy;

        //init W
        Eigen::Matrix<double,3,3> Wx;
        Eigen::Matrix<double,3,3> Wy;
        double q11 = (2*pow(alefa,3)*pow(T,3) -
                    6*pow(alefa,2)*pow(T,2) +
                    6*alefa*T + 3 -
                    12*alefa*T*exp(-alefa*T) -
                    3*exp(-2*alefa*T))/
                    (6*pow(alefa,5));

        double q12 = (pow(alefa,2)*pow(T,2) -
                    2*alefa*T + 1 -
                    2*exp(-alefa*T) +
                    exp(-2*alefa*T) +
                    2*alefa*T*exp(-alefa*T))/
                    (2*pow(alefa,4));

        double q13 = (1 - 2*alefa*T*exp(-alefa*T) -
                    exp(-2*alefa*T))/
                    (2*pow(alefa,3));

        double q21 = q12;

        double q22 = (2*alefa*T - 3 +
                    4*exp(-alefa*T) -
                    exp(-2*alefa*T))/
                    (2*pow(alefa,3));

        double q23 = (1 - 2*exp(-alefa*T) +
                    exp(-2*alefa*T))/
                    (2*pow(alefa,2));

        double q31 = q13;

        double q32 = q23;

        double q33 = (1 - exp(-2*alefa*T))/(2*alefa);

        Wx << q11,q12,q13,
                q21,q22,q23,
                q31,q32,q33;
        Wy = Wx;

        W << 2*alefa*Sigmaq*Wx , Eigen::Matrix<double,3,3>::Zero(),
                Eigen::Matrix<double,3,3>::Zero() , 2*alefa*Sigmaq*Wy;

        //init G
        Eigen::Matrix<double,3,1> Gx,Gy;
        Gx << (-T + alefa*T*T/2 + (1-exp(-alefa*T))/alefa)/alefa,
                T - (1 - exp(-alefa*T))/alefa,
                1 - exp(-alefa*T);
        Gy = Gx;

        G<<Gx,Eigen::Matrix<double,3,1>::Zero(),
                Eigen::Matrix<double,3,1>::Zero(),Gy;

    //    std::cout<<"F_Data:"<<F<<std::endl;
    }
}