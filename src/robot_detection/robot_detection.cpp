//ros
#include <ros/ros.h>
// #include <camera_subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//eigen
#include <Eigen/Dense>
//c++
#include <iostream>

#include "armor_detection.hpp"
#include "armor_track.h"

cv::Mat src;

robot_detection::ArmorDetector autoShoot;
std::vector<robot_detection::Armor> autoTarget;

clock_t image_timestamp_;
clock_t image_last_timestamp_;

void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    src = cv_bridge::toCvCopy(msg, "bgr8")->image.clone();
    // ROS_INFO("show miage's width %d \n", src.cols);

    cv::imshow("main-result-image", src);
    cv::waitKey(5);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  image_timestamp_ = std::clock();
  double delta_t = (double)(image_timestamp_ - image_last_timestamp_) / CLOCKS_PER_SEC;

  ROS_INFO("FPS %lf \n", 1/delta_t);

  autoTarget = autoShoot.autoAim(src);
  if (!autoTarget.empty())
  {
    std::cout<<"main get ---"<<autoTarget.size()<<"--- target!!!"<<std::endl;
  }
  else
  {
    std::cout<<"no target!!!"<<std::endl;
  }
  

  image_last_timestamp_ = image_timestamp_;
}







int main(int argc, char  *argv[])
{
  setlocale(LC_ALL,"");
  ros::init(argc,argv,"mv_camera_sub");
  ros::NodeHandle nh;


  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_ = it.subscribe("image_raw",1,ImageCallback);
    

  




  ros::spin();
  return 0;
}
