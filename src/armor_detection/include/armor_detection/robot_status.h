//opencv
#include <opencv2/opencv.hpp>
//ros
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//basic parameter
#define GRAVITY 9.78
#define SMALL_AIR_K 0.01903
#define BIG_AIR_K 0.00556
#define BIG_LIGHT_AIR_K 0.00530

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
    float ab_pitch = 0.0;
    float ab_yaw = 0.0;
    float ab_roll = 0.0;
    float SPEED = 25.0;
    int enermy_color = BLUE;
    int enermy_type;
    int enermy_ID;
};