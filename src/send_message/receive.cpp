#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"

#include "robot_msgs/vision.h"
#include "ros/ros.h"

void doMsg(const robot_msgs::visionConstPtr& vision_msg){

    robot_msgs::vision vision_data;
    vision_data = *vision_msg;
    int id = vision_data.id;
    float roll = vision_data.roll;
    float pitch = vision_data.pitch;
    float yaw = vision_data.yaw;
    float bullet_speed = vision_data.shoot;
    int mode = vision_data.shoot_sta;
}

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"vision_sub");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<robot_msgs::vision>("vision_data",10,doMsg);

    ros::spin();
    return 0;
}