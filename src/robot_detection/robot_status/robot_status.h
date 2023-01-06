#pragma once
//opencv
#include <opencv2/opencv.hpp>

//basic parameter
#define GRAVITY 9.78
#define SMALL_AIR_K 0.01903
#define BIG_AIR_K 0.00556
#define BIG_LIGHT_AIR_K 0.00530

namespace robot_detection {
    //robot basic classes
    enum EnemyColor { RED = 1, BLUE = 2 };
    enum EnemyType  { SMALL = 1, BIG = 2 };
    enum EnemyState { RUN = 1, SPIN = 2};
    enum SpinHeading { UNKNOWN, CLOCKWISE, COUNTER_CLOCKWISE };

    //robot state information from electronic control group
    class robot_state
    {
    public:
        //电控发来的角度和弹速
        float ab_pitch;
        float ab_yaw;
        float ab_roll;
        float bullet_speed;
        int enemy_color;

        robot_state() = default;

        void clone(robot_state &robot);

        void updateData(float data[4]);
        void updateData(float data[4], int color);
    };

}