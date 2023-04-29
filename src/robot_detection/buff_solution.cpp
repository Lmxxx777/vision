//ros
#include <ros/ros.h>
#include <ros/time.h>
// #include <camera_subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CompressedImage.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
// msg
#include "robot_msgs/vision.h"
#include "robot_msgs/robot_ctrl.h"
#include "robot_msgs/EulerAngles.h"
//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//eigen
#include <Eigen/Dense>
//matplotlib
// #include "matplotlib.h"
//ceres
// #include "ceres/ceres.h"
//c++
#include <iostream>
//vision
#include "buff_detection.h"


cv::Mat src;

robot_detection::BuffDetector buff_detection;

// robot_detection::AngleSolve AS;  // for test

// 0~pi
double last_pitch, last_yaw;
double delta_pitch, delta_yaw;
double gimbal_pitch, gimbal_yaw, gimbal_roll;
double offset_x, offset_y, offset_z;

ros::Publisher vision_pub_;

// clock_t
// clock_t image_timestamp_;
// clock_t image_last_timestamp_;

// //Time
// ros::Time begin;
// ros::Time end;

void callback(const sensor_msgs::ImageConstPtr & src_msg, const robot_msgs::visionConstPtr &imu_msg)
{
    //  ---   calculate time   ---
    // Time for fps
    ros::Time begin = ros::Time::now();

    // begin for predict
    robot_detection::chrono_time now_time = std::chrono::high_resolution_clock::now();

    // camera
    src = cv_bridge::toCvCopy(src_msg, "bgr8")->image;
    // ROS_INFO("show miage's width %d \n", src.cols);

    // imu
    robot_msgs::vision vision_data;
    vision_data = *imu_msg;
    int enemy_color = vision_data.id;
    float roll = vision_data.roll;
    float pitch = vision_data.pitch;
    float yaw = vision_data.yaw;
    float quaternion[4] = {
        vision_data.quaternion[0],
        vision_data.quaternion[1],
        vision_data.quaternion[2],
        vision_data.quaternion[3],
    };
    float bullet_speed = vision_data.shoot_spd;
    int mode = vision_data.shoot_sta;

    bullet_speed = 28;
    // TODO: 先验证数据的情况
    // ROS_INFO("gimbal_roll  is  %lf \n", roll);
    // ROS_INFO("gimbal_pitch is  %lf \n", pitch);
    // ROS_INFO("gimbal_yaw   is  %lf \n", yaw);
    // ROS_INFO("enemy_color  is  %d  \n", enemy_color);
    // ROS_INFO("bullet_speed is  %lf \n", bullet_speed);
    // ROS_INFO("mode         is  %x  \n", mode);
    buff_detection.AS.init(roll, pitch, yaw, quaternion, bullet_speed);

    // buff-detecting
    buff_detection.detectResult(src, now_time);


    // Time
    ros::Time end = ros::Time::now();
    ros::Duration duration_t = end - begin;
    double delta_tt = duration_t.toSec();
    cv::putText(src,"FPS      : "+std::to_string(1/delta_tt),cv::Point2f(0,30),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    // ROS_INFO("FPS %lf \n", 1/delta_tt);

    // show image
    cv::imshow("main-result-image", src);
    cv::waitKey(1);
}

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"mv_camera_sub");
    ros::NodeHandle nh;
    vision_pub_ = nh.advertise<robot_msgs::robot_ctrl>("robot_ctrl", 1);

    image_transport::ImageTransport it(nh);

    message_filters::Subscriber<sensor_msgs::Image> camera_src_sub(nh, "image_raw", 1);  
    message_filters::Subscriber<robot_msgs::vision> camera_imu_sub(nh, "vision_data", 1);  
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, robot_msgs::vision> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), camera_src_sub, camera_imu_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    return 0;
}
