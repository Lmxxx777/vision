#include "robot_status.h"

namespace robot_detection {

robot_state::robot_state()
{
    enermy_color = BLUE;
    enermy_type = SMALL;
}

void robot_state::initSrc(cv::Mat Src_)
{
    src=Src_.clone();
}

void robot_state::initPose(float p, float y, float r)
{
    ab_pitch = p;
    ab_yaw = y;
    ab_roll = r;
}

void robot_state::initSpeed(float speed)
{
    bullet_speed = speed;
}

}