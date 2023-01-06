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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

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
#include "ceres/ceres.h"
//c++
#include <iostream>
#include "armor_track.h"


cv::Mat src;

robot_detection::ArmorDetector Detect;
std::vector<robot_detection::Armor> Targets;

robot_detection::ArmorTracker Track;

robot_detection::AngleSolve AS;  // for test

// 0~pi
double last_pitch, last_yaw;
double delta_pitch, delta_yaw;
double gimbal_pitch, gimbal_yaw, gimbal_roll;
double offset_x, offset_y, offset_z;
bool first = false;
double r,p,y;

// clock_t
// clock_t image_timestamp_;
// clock_t image_last_timestamp_;

// //Time
// ros::Time begin;
// ros::Time end;

void ImageCallback(const sensor_msgs::ImageConstPtr& src_msg/*, const sensor_msgs::CameraInfoConstPtr& src_info*/) //, const sensor_msgs::CameraInfo& info
{

  try
  {
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(src_msg, "bgr8");
    src = cv_ptr_compressed->image;
    cv::imshow("main-result-image", src);
    cv::waitKey(5);


    // src = cv_bridge::toCvCopy(src_msg, "bgr8")->image.clone();
    // ROS_INFO("show miage's width %d \n", src.cols);
    // cv::imshow("main-result-image", src);
    // cv::waitKey(5);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", src_msg->encoding.c_str());
  }

  
  // clock_t
  // image_timestamp_ = std::clock();
  // double delta_t = (double)(image_timestamp_ - image_last_timestamp_) / CLOCKS_PER_SEC;
  // ROS_INFO("FPS %lf \n", 1/delta_t);

  // Time
  // end = ros::Time::now();
  // ros::Duration duration = end - begin;
  // double delta_tt = duration.toSec();
  // ROS_INFO("FPS %lf \n", 1/delta_tt);

  // Targets = Detect.autoAim(src);
  // if (!Targets.empty())
  // {
  //   std::cout<<"main get ---"<<Targets.size()<<"--- target!!!"<<std::endl;
  // }
  // else
  // {
  //   std::cout<<"no target!!!"<<std::endl;
  // }
  
  // // clock_t
  // image_last_timestamp_ = image_timestamp_;

  // Time
  // begin = end;
}

void ImuCallback(const robot_msgs::EulerAnglesConstPtr &imu_msg)
{
  robot_msgs::EulerAngles Euler_angle;
  Euler_angle = *imu_msg;
  // *imu_msg.unique
  gimbal_pitch = Euler_angle.pitch;
  gimbal_yaw = Euler_angle.yaw;
  gimbal_roll = Euler_angle.roll;
  
  ROS_INFO("gimbal_pitch is  %lf \n", gimbal_pitch);
  ROS_INFO("gimbal_yaw   is  %lf \n", gimbal_yaw);
  ROS_INFO("gimbal_roll  is  %lf \n", gimbal_roll);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    ROS_INFO("imu: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
             msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
             msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
}

void callback(const sensor_msgs::ImageConstPtr & src_msg, const robot_msgs::EulerAnglesConstPtr &imu_msg)
{
  //  ---   calculate time   ---
  // Time
  ros::Time begin = ros::Time::now();

  // begin
  double now_time = (double)cv::getTickCount();
  
  // camera
  src = cv_bridge::toCvCopy(src_msg, "bgr8")->image;
  // ROS_INFO("show miage's width %d \n", src.cols);

  // imu
  robot_msgs::EulerAngles Euler_angle;
  Euler_angle = *imu_msg;
  // *imu_msg.unique
  gimbal_roll = Euler_angle.roll/CV_PI*180;
  gimbal_pitch = Euler_angle.pitch/CV_PI*180;
  gimbal_yaw = Euler_angle.yaw/CV_PI*180;
  // ROS_INFO("gimbal_roll  is  %lf \n", gimbal_roll);
  // ROS_INFO("gimbal_pitch is  %lf \n", gimbal_pitch);
  // ROS_INFO("gimbal_yaw   is  %lf \n", gimbal_yaw);

  int enemy_color = 1; // red-1 blue-2
  float bullet_speed = 28.0;

  // detecting
  Targets = Detect.autoAim(src, enemy_color);
  if (!Targets.empty())
  {
    // std::cout<<"main get ---"<<Targets.size()<<"--- target!!!"<<std::endl;
  }
  else
  {
    // std::cout<<"no target!!!"<<std::endl;
  }

  // tracking
  Track.AS.init(gimbal_roll, gimbal_pitch, gimbal_yaw, bullet_speed);
  bool track_bool = Track.locateEnemy(src,Targets,now_time);
  if(track_bool)
  {
    std::cout<<"track!!!"<<Track.tracker_state<<"  id: "<<Track.tracking_id<<std::endl;
  }
  else
  {
    std::cout<<"loss!!!"<<std::endl;
  }


  // Time
  ros::Time end = ros::Time::now();
  ros::Duration duration_t = end - begin;
  double delta_tt = duration_t.toSec();
  cv::putText(src,"FPS: "+std::to_string(delta_tt),cv::Point2f(0,30),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
  ROS_INFO("FPS %lf \n", 1/delta_tt);

  // show image
  cv::imshow("main-result-image", src);
  cv::waitKey(1);
}

int main(int argc, char  *argv[])
{
  setlocale(LC_ALL,"");
  ros::init(argc,argv,"mv_camera_sub");
  ros::NodeHandle nh;


  image_transport::ImageTransport it(nh);
  // image_transport::Subscriber camera_src_sub = it.subscribe("image_raw",1,ImageCallback);  //,image_transport::TransportHints("compressed")
  // ros::Subscriber camera_imu_sub = nh.subscribe("euler_angles",1,ImuCallback);
  // ros::Subscriber imu_sub = nh.subscribe("imu",1,imuCallback);

  message_filters::Subscriber<sensor_msgs::Image> camera_src_sub(nh, "image_raw", 1);  
  message_filters::Subscriber<robot_msgs::EulerAngles> camera_imu_sub(nh, "euler_angles", 1);  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, robot_msgs::EulerAngles> MySyncPolicy;
  // // // // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), camera_src_sub, camera_imu_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));


  ros::spin();
  return 0;
}
