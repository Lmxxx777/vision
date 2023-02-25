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

#include "robot_msgs/vision.h"
#include "robot_msgs/robot_ctrl.h"
// #include "robot_msgs/EulerAngles.h"
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


ros::Publisher vision_pub_;

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
/*
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
*/
void callback2(const sensor_msgs::ImageConstPtr & src_msg, const robot_msgs::visionConstPtr &imu_msg)
{
  //  ---   calculate time   ---
  // Time for fps
  ros::Time begin = ros::Time::now();

  // begin for predict
  double now_time = (double)cv::getTickCount();
  
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
  float bullet_speed = 28;
  int mode = vision_data.shoot_sta;
  
  // ROS_INFO("gimbal_roll  is  %lf \n", roll);
  // ROS_INFO("gimbal_pitch is  %lf \n", pitch);
  // ROS_INFO("gimbal_yaw   is  %lf \n", yaw);
  // ROS_INFO("enemy_color  is  %d  \n", enemy_color);
  // ROS_INFO("bullet_speed is  %lf \n", bullet_speed);
  // ROS_INFO("mode         is  %x  \n", mode);

  // detecting
  Targets = Detect.autoAim(src, enemy_color);
  if (!Targets.empty())
  {
    std::cout<<"main get ---"<<Targets.size()<<"--- target!!!"<<std::endl;
  }
  else
  {
    std::cout<<"no target!!!"<<std::endl;
  }

  // tracking
  robot_msgs::robot_ctrl send_data;
  Track.AS.init(roll, pitch, yaw, bullet_speed);
  bool track_bool = Track.locateEnemy(src,Targets,now_time);
  // track_bool = false;
  if(track_bool)
  {
    send_data.pitch = Track.pitch;
    send_data.yaw = Track.yaw;
    std::cout<<"track!!!"<<Track.tracker_state<<"  id: "<<Track.tracking_id<<std::endl;
  }
  else
  {
    send_data.pitch = Track.pitch;
    send_data.yaw = Track.yaw;
    // send_data.pitch = Track.AS.ab_pitch;
    // send_data.yaw = Track.AS.ab_yaw;
    std::cout<<"loss!!!"<<std::endl;
  }

  // send_data.pitch = 12;
  // send_data.yaw = 34;
  std::cout<<send_data.pitch<<"        "<<send_data.yaw<<std::endl;
  // send message
  vision_pub_.publish(send_data);
  cv::putText(src,"PITCH    : "+std::to_string(send_data.pitch),cv::Point2f(0,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
  cv::putText(src,"YAW      : "+std::to_string(send_data.yaw),cv::Point2f(0,90),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
  cv::putText(src,"DISTANCE : "+std::to_string(Track.enemy_armor.camera_position.norm())+"m",cv::Point2f(0,120),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255,255,0),1,3);

  cv::Point2f vertice_lights[4];
  Track.enemy_armor.points(vertice_lights);
  for (int i = 0; i < 4; i++) {
    line(src, vertice_lights[i], vertice_lights[(i + 1) % 4], CV_RGB(0, 255, 255),2,cv::LINE_8);
  }
    std::string information = std::to_string(Track.enemy_armor.id) + ":" + std::to_string(Track.enemy_armor.confidence*100) + "%";
//        putText(final_armors_src,ff,finalArmors[i].center,FONT_HERSHEY_COMPLEX, 1.0, Scalar(12, 23, 200), 1, 8);
                putText(src, information,Track.enemy_armor.armor_pt4[3],cv::FONT_HERSHEY_COMPLEX,2,cv::Scalar(255,0,255),1,3);



  // Time
  ros::Time end = ros::Time::now();
  ros::Duration duration_t = end - begin;
  double delta_tt = duration_t.toSec();
  cv::putText(src,"FPS      : "+std::to_string(1/delta_tt),cv::Point2f(0,30),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
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
  vision_pub_ = nh.advertise<robot_msgs::robot_ctrl>("robot_ctrl", 1);

  image_transport::ImageTransport it(nh);
  // image_transport::Subscriber camera_src_sub = it.subscribe("image_raw",1,ImageCallback);  //,image_transport::TransportHints("compressed")
  // ros::Subscriber camera_imu_sub = nh.subscribe("euler_angles",1,ImuCallback);
  // ros::Subscriber imu_sub = nh.subscribe("imu",1,imuCallback);

  message_filters::Subscriber<sensor_msgs::Image> camera_src_sub(nh, "image_raw", 1);  
  // // fdilink_ahrs
  // message_filters::Subscriber<robot_msgs::EulerAngles> camera_imu_sub(nh, "euler_angles", 1);  
  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, robot_msgs::EulerAngles> MySyncPolicy;
  // // c 板
  message_filters::Subscriber<robot_msgs::vision> camera_imu_sub(nh, "vision_data", 1);  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, robot_msgs::vision> MySyncPolicy;
  // // // // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), camera_src_sub, camera_imu_sub);
  sync.registerCallback(boost::bind(&callback2, _1, _2));

  ros::spin();
  return 0;
}
