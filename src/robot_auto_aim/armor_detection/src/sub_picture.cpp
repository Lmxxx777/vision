

//opencv
#include <opencv2/opencv.hpp>
//ros
#include <image_transport/image_transport.h>
// #include <camera_subscriber.h>
#include <cv_bridge/cv_bridge.h>

#include "std_msgs/String.h"

void callback(const sensor_msgs::ImageConstPtr& msg)
{
    try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    //2.初始化 ROS 节点:命名(唯一)
    ros::init(argc,argv,"picture_listener");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;

    //4.实例化 订阅者 对象
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub_ = it.subscribe("image_raw",5,callback);
    //ros::Subscriber sub = nh.subscribe<sensor_msgs::>("chatter",10,doMsg);
    //5.处理订阅的消息(回调函数)

    //     6.设置循环调用回调函数
    ros::spin();//循环读取接收的数据，并调用回调函数处理

    return 0;
}
