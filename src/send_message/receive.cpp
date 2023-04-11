#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <Eigen/Dense>
// #include "Eigen/eigen.hpp"

#include "robot_msgs/vision.h"
#include "ros/ros.h"

    Eigen::Matrix3d quaternionToRotationMatrix(float quaternion[4])
    {
        Eigen::Matrix3d R_x;
        float w=quaternion[0],x=quaternion[1],y=quaternion[2],z=quaternion[3];
        R_x << 1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w,
                2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w,
                2*x*z-2*y*w, 2*y*z+2*w*x, 1-2*x*x-2*y*y;

        float roll = atan2(2*y*z + 2*w*x,1 - 2*x*x - 2*y*y)/3.14 * 180.0f;
        float pitch = asin(2*w*y - 2*x*z)/3.14*180.0f;
        float yaw = atan2(2*x*y + 2*w*z, 1 - 2*y*y - 2*z*z)/3.14*180.0f;

        // std::cout<<"----------[quaternion_euler]-----------"<<std::endl;
        // std::cout<<"[roll:]   |"<<roll<<std::endl;
        // std::cout<<"[pitch:]  |"<<pitch<<std::endl;
        // std::cout<<"[yaw:]    |"<<yaw<<std::endl;

        return R_x;
    }

void doMsg(const robot_msgs::visionConstPtr& vision_msg){

    robot_msgs::vision vision_data;
    vision_data = *vision_msg;
    int id = vision_data.id;
    float roll = vision_data.roll;
    float pitch = vision_data.pitch;
    float yaw = vision_data.yaw;
    float quaternion[4];
    for(int i = 0; i<vision_data.quaternion.size(); ++i)
        quaternion[i] = vision_data.quaternion[i];
    float bullet_speed = vision_data.shoot_spd;
    int mode = vision_data.shoot_sta;

    quaternionToRotationMatrix(quaternion);

    // ROS_INFO("gimbal_roll  is  %lf \n", roll);
    // ROS_INFO("gimbal_pitch is  %lf \n", pitch);
    // ROS_INFO("gimbal_yaw   is  %lf \n", yaw);
    // ROS_INFO("enemy_color  is  %d  \n", enemy_color);
    // ROS_INFO("bullet_speed is  %lf \n", bullet_speed);
    // ROS_INFO("mode         is  %x  \n", mode);

    // float q[4] = {1, -0.00, 0.00, 0.00};
    float q[4] = {0.9999614953994751, -0.004679855890572071, 0.007264930289238691, 0.0015828351024538279};
    Eigen::Matrix3d rotate_mat = quaternionToRotationMatrix(q);

    Eigen::Vector3d tmp_pos = {2.5,0.3,-0.3};
    Eigen::Vector3d imu_pos = rotate_mat * tmp_pos;
    std::cout<<"imu_pos:   "<<imu_pos.transpose()<<std::endl;
    Eigen::Vector3d rpy;
    rpy[0] = 0;
    rpy[1] = atan2(imu_pos[2],imu_pos[0]) / 3.1415926535*180.0;
    rpy[2] = atan2(imu_pos[1],imu_pos[0]) / 3.1415926535*180.0;

        rpy[2] = -atan2(imu_pos[0],imu_pos[1]) / 3.14*180.0;
        rpy[1] = atan2(imu_pos[2],imu_pos[1]) / 3.14*180.0;
        // rpy[1] = 0;
        rpy[0] = atan2(imu_pos[2],imu_pos[0]) / 3.14*180.0;
    std::cout<<"rpy:   "<<rpy.transpose()<<std::endl;
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