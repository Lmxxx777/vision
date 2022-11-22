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
enum EnermyColor { RED = 1, BLUE = 2 };
enum EnermyType  { SMALL = 1, BIG = 2 };
enum EnermyState { RUN = 1, SPIN = 2};
enum SpinHeading { UNKNOWN, CLOCKWISE, COUNTER_CLOCKWISE };

//robot state information from electronic control group
class robot_state
{
public:
    cv::Mat src;

    //电控发来的角度和弹速
    float ab_pitch;
    float ab_yaw;
    float ab_roll;
    float bullet_speed;
    int enermy_color;
    int enermy_type;


    robot_state();
    void initSrc(cv::Mat Src_);
    void initPose(float p, float y, float r);
    void initSpeed(float speed);
};

}