#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"

#include "robot_msgs/vision.h"

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"vision_pub");
    ros::NodeHandle nh;
    ros::Publisher vision_pub_;

    vision_pub_ = nh.advertise<robot_msgs::vision>("vision_data", 1);

    robot_msgs::vision vision_data;
    vision_data.header.frame_id = "vision_data";
    vision_data.header.seq++;
    vision_data.header.stamp = ros::Time::now();
    vision_data.id = 1;
    vision_data.roll = 0.0;
    vision_data.pitch = 0.0;
    vision_data.yaw = 0.0;
    vision_data.shoot = 28.0;
    vision_data.shoot_sta = 0x21;

    // Time
    ros::Time begin;
    ros::Time end;

    ros::Rate r(100);

    while (ros::ok())
    {
        // Time
        end = ros::Time::now();
        ros::Duration duration = end - begin;
        double delta_tt = duration.toSec();
        ROS_INFO("imu_FPS %lf \n", 1/delta_tt);
        begin = end;

        vision_pub_.publish(vision_data);

        r.sleep();
        ros::spinOnce();
    }
}