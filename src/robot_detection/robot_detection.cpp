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
#include "armor_track.h"


cv::Mat src;

robot_detection::ArmorDetector Detect;
std::vector<robot_detection::Armor> Targets;

robot_detection::ArmorTracker Track;

// robot_detection::AngleSolve AS;  // for test

// 0~pi
double last_pitch, last_yaw;
double delta_pitch, delta_yaw;
double gimbal_pitch, gimbal_yaw, gimbal_roll;
double offset_x, offset_y, offset_z;

ros::Publisher vision_pub_;
ros::Publisher aim_point_pub_;

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
    // robot_msgs::EulerAngles Euler_angle;
    // Euler_angle = *imu_msg;
    // // *imu_msg.unique
    // gimbal_pitch = Euler_angle.pitch;
    // gimbal_yaw = Euler_angle.yaw;
    // gimbal_roll = Euler_angle.roll;

    // ROS_INFO("gimbal_pitch is  %lf \n", gimbal_pitch);
    // ROS_INFO("gimbal_yaw   is  %lf \n", gimbal_yaw);
    // ROS_INFO("gimbal_roll  is  %lf \n", gimbal_roll);
}

void C_callback(const robot_msgs::visionConstPtr &imu_msg)
{
    // robot_msgs::EulerAngles Euler_angle;
    // Euler_angle = *imu_msg;
    // // *imu_msg.unique
    // gimbal_pitch = Euler_angle.pitch;
    // gimbal_yaw = Euler_angle.yaw;
    // gimbal_roll = Euler_angle.roll;

    // ROS_INFO("gimbal_pitch is  %lf \n", gimbal_pitch);
    // ROS_INFO("gimbal_yaw   is  %lf \n", gimbal_yaw);
    // ROS_INFO("gimbal_roll  is  %lf \n", gimbal_roll);
}

void callback(const sensor_msgs::ImageConstPtr & src_msg, const robot_msgs::visionConstPtr &imu_msg)
{
    //  ---   calculate time   ---
    // Time for fps
    ros::Time begin = ros::Time::now();

    // begin for predict
    robot_detection::chrono_time now_time = std::chrono::high_resolution_clock::now();
    // robot_detection::chrono_time t;

    // camera
    src = cv_bridge::toCvCopy(src_msg, "bgr8")->image;
    // ROS_INFO("show miage's width %d \n", src.cols);

    cv::Mat _src = src.clone();

    // imu
    robot_msgs::vision vision_data;
    vision_data = *imu_msg;
    int enemy_color = vision_data.id;  // TODO: RED 7-me; BLUE 107
    float roll = vision_data.roll;
    float pitch = vision_data.pitch;
    float yaw = vision_data.yaw;
    float quaternion[4];
    for(int i = 0; i<vision_data.quaternion.size(); ++i)
    {
        quaternion[i] = vision_data.quaternion[i];
        // quaternion[i] = round(quaternion[i] * 1000)/1000;   // TODO: jieduan shuju
        // std::cout<<quaternion[i]<<" ";
    }
    // std::cout<<std::endl;
    float bullet_speed = vision_data.shoot_spd;
    if(bullet_speed <= 0)
    {
        bullet_speed = 27;
    }
    int mode = vision_data.shoot_sta;   // TODO: 

    // TODO: 先验证数据的情况
    // ROS_INFO("gimbal_roll  is  %lf \n", roll);
    // ROS_INFO("gimbal_pitch is  %lf \n", pitch);
    // ROS_INFO("gimbal_yaw   is  %lf \n", yaw);
    // ROS_INFO("enemy_color  is  %d  \n", enemy_color);
    // ROS_INFO("bullet_speed is  %lf \n", bullet_speed);
    // ROS_INFO("mode         is  %x  \n", mode);

    // TODO: enemy color set 1-RED 2-BLUE
    // enemy_color = 2; 
    int color;
    // std::cout<<"enemy_color:  "<<enemy_color<<std::endl;
    if(enemy_color == 7)
    {
        color = robot_detection::BLUE;
    }
    else if(enemy_color == 107)
    {
        color = robot_detection::RED;
    }

    // detecting
    Targets = Detect.autoAim(src, color);
    if (!Targets.empty())
    {
        // std::cout<<"main get ---"<<Targets.size()<<"--- target!!!"<<std::endl;
    }
    else
    {
        // std::cout<<"no target!!!"<<std::endl;
    }
    cv::putText(src,std::to_string(Targets.size()) +" ARMOR",cv::Point2f(1280 - 200,30),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(src,"SPEED  " + std::to_string(bullet_speed),cv::Point2f(0,512),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    
    cv::putText(src,"0  " + std::to_string(quaternion[0]),cv::Point2f(0,512+30),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(src,"1  " + std::to_string(quaternion[1]),cv::Point2f(0,512+60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(src,"2  " + std::to_string(quaternion[2]),cv::Point2f(0,512+90),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(src,"3  " + std::to_string(quaternion[3]),cv::Point2f(0,512+120),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    
    // tracking
    // port & aim_point
    robot_msgs::robot_ctrl vision_send_data;
    geometry_msgs::PointStamped aim_point;
 
    Track.AS.init(roll, pitch, yaw, quaternion, bullet_speed);
    bool track_bool = false;

    // 不开自瞄一直重置跟踪器
    // bool is_need_reset = mode != 0x31 ? true : false;
    // is_need_reset = false;  // TODO:  for test
    // if(is_need_reset)
    //     Track.reset();

    // for(int i=0; i<Targets.size(); ++i)
    // {
    //     std::cout<<Targets[i].id<<" * ";
    // }
    // std::cout<<std::endl;

    track_bool= Track.locateEnemy(src,Targets,now_time);

    if(track_bool)
    {
        mode = 1;
        // std::cout<<"track!!!"<<Track.tracker_state<<"  id: "<<Track.tracking_id<<std::endl;
        if(mode)
        {
            vision_send_data.fire_command = 0x31;
            vision_send_data.target_lock = 0x31;
            // transform.setOrigin(tf::Vector3(Track.enemy_armor.camera_position[0],Track.enemy_armor.camera_position[1],Track.enemy_armor.camera_position[2]));
        }
        if(Track.tracker_state == robot_detection::LOSING)
        {
            vision_send_data.fire_command = 0x32;
            vision_send_data.target_lock = 0x31;
            // transform.setOrigin(tf::Vector3(0,0,0));
        }
        vision_send_data.aim_id = Track.tracking_id;
        vision_send_data.pitch = Track.pitch;
        vision_send_data.yaw = Track.yaw;
        last_pitch = Track.pitch;
        last_yaw = Track.yaw;
        cv::circle(src,cv::Point(640,20),20,cv::Scalar(0,255,0),-1);
    }
    else
    {
        // std::cout<<"loss!!!"<<std::endl;
        vision_send_data.fire_command = 0x32;
        vision_send_data.target_lock = 0x32;
        // vision_send_data.pitch = Track.pitch;
        // vision_send_data.yaw = Track.yaw;
        // transform.setOrigin(tf::Vector3(0,0,0));
        vision_send_data.aim_id = 0;
        vision_send_data.pitch = Track.AS.ab_pitch;
        vision_send_data.yaw = Track.AS.ab_yaw;

        vision_send_data.pitch = last_pitch;
        vision_send_data.yaw = last_yaw;
        cv::circle(src,cv::Point(640,20),20,cv::Scalar(0,0,255),-1);
    }

    // vision_send_data.fire_command = 0x32;
    // vision_send_data.target_lock = 0x32;

    // send port gimbal message 
    vision_pub_.publish(vision_send_data);  
    // send aim point in camera
    aim_point.header.frame_id = "camera";
    aim_point.header.seq++;
    aim_point.header.stamp = ros::Time::now();
    aim_point.point.x = Track.enemy_armor.camera_position[0];
    aim_point.point.y = Track.enemy_armor.camera_position[1];
    aim_point.point.z = Track.enemy_armor.camera_position[2];
    // aim_point_pub_.publish(aim_point);

    // show track state on img's ru
    switch (Track.tracker_state)
    {
    case 0: // MISSING
        cv::putText(src,"MISSING   ", cv::Point2f(1280 - 200,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        break;
    case 1: // DETECTING
        cv::putText(src,"DETECTING " + std::to_string(Track.tracking_id),cv::Point2f(1280 - 200,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        break;
    case 2: // LOSING
        cv::putText(src,"LOSING    " + std::to_string(Track.tracking_id),cv::Point2f(1280 - 200,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        break;
    case 3: // TRACKING
        cv::putText(src,"TRACKING  " + std::to_string(Track.tracking_id),cv::Point2f(1280 - 200,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        break;
    }

    // show receive data on img's ru
    cv::putText(src,"p : "+std::to_string(pitch),cv::Point2f(1280 - 300,90),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(src,"y : "+std::to_string(yaw),cv::Point2f(1280 - 300,120),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(src,"r : "+std::to_string(roll),cv::Point2f(1280 - 300,150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);

    // show send data on img's lu
    cv::putText(src,"PITCH    : "+std::to_string(Track.pitch),cv::Point2f(0,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(src,"YAW      : "+std::to_string(Track.yaw),cv::Point2f(0,90),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(src,"ROLL     : "+std::to_string(Track.roll),cv::Point2f(0,120),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(src,"DISTANCE : "+std::to_string(Track.enemy_armor.camera_position.norm())+"m",cv::Point2f(0,150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);

    // show delta angle to rotate 
    cv::putText(src,"delta_p : "+std::to_string(Track.pitch - Track.AS.ab_pitch),cv::Point2f(1280 - 300,512),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(src,"delta_y : "+std::to_string(Track.yaw - Track.AS.ab_yaw),cv::Point2f(1280 - 300,512 + 30),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    
    // 展示选择到的装甲板
    cv::Point2f vertice_enemy[4];
    Track.enemy_armor.points(vertice_enemy);
    for (int i = 0; i < 4; i++) {
        line(src, vertice_enemy[i], vertice_enemy[(i + 1) % 4], CV_RGB(255, 0, 255),2,cv::LINE_8);
    }
    std::string information = std::to_string(Track.enemy_armor.id) + ":" + std::to_string(Track.enemy_armor.confidence*100) + "%";
    putText(src, information,Track.enemy_armor.armor_pt4[3],cv::FONT_HERSHEY_SIMPLEX,2,CV_RGB(255,0,255),1,3);

    // 展示要匹配的预测装甲板
    cv::Point2f armor_match_center = Track.AS.imu2pixel(Track.predicted_enemy.head(3));
    // cv::circle(src,armor_match_center,5,cv::Scalar(255,0,0),-1);
    cv::RotatedRect armor_match = cv::RotatedRect(armor_match_center,
                                                cv::Size2f(Track.enemy_armor.size.width,Track.enemy_armor.size.height),
                                                Track.enemy_armor.angle);

    cv::Point2f vertice_armor_match[4];
    Track.enemy_armor.points(vertice_armor_match);
    for (int i = 0; i < 4; ++i) {
        line(src, vertice_armor_match[i], vertice_armor_match[(i + 1) % 4], CV_RGB(0, 255, 0),2,cv::LINE_8);
    }

    cv::putText(src,"camera_position",cv::Point2f(0,1024 - 150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(src,"x  : "+std::to_string(Track.enemy_armor.camera_position[0]),cv::Point2f(0,1024 - 120),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(src,"y  : "+std::to_string(Track.enemy_armor.camera_position[1]),cv::Point2f(0,1024 - 90),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(src,"z  : "+std::to_string(Track.enemy_armor.camera_position[2]),cv::Point2f(0,1024 - 60),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    
    cv::Point2f bullet = Track.AS.imu2pixel(Track.bullet_point);
    cv::circle(src,bullet,3,cv::Scalar(150,100,255),-1);

    cv::putText(src,"predict_position",cv::Point2f(1280 - 900,1024 - 150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(src,"x  : "+std::to_string(Track.predicted_position[0]),cv::Point2f(1280 - 900,1024 - 120),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(src,"y  : "+std::to_string(Track.predicted_position[1]),cv::Point2f(1280 - 900,1024 - 90),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(src,"z  : "+std::to_string(Track.predicted_position[2]),cv::Point2f(1280 - 900,1024 - 60),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);

    cv::putText(src,"bullet_position",cv::Point2f(1280 - 600,1024 - 150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(src,"x  : "+std::to_string(Track.bullet_point[0]),cv::Point2f(1280 - 600,1024 - 120),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(src,"y  : "+std::to_string(Track.bullet_point[1]),cv::Point2f(1280 - 600,1024 - 90),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(src,"z  : "+std::to_string(Track.bullet_point[2]),cv::Point2f(1280 - 600,1024 - 60),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);

    cv::putText(src,"world_position",cv::Point2f(1280 - 300,1024 - 150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(src,"x  : "+std::to_string(Track.enemy_armor.world_position[0]),cv::Point2f(1280 - 300,1024 - 120),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(src,"y  : "+std::to_string(Track.enemy_armor.world_position[1]),cv::Point2f(1280 - 300,1024 - 90),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(src,"z  : "+std::to_string(Track.enemy_armor.world_position[2]),cv::Point2f(1280 - 300,1024 - 60),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    

    // 用预测位置为中心点，选择的装甲板画框
    cv::Point2f armor_singer_center = Track.AS.imu2pixel(Track.predicted_position);
    // armor_singer_center.y = armor_match_center.y;
    cv::RotatedRect armor_singer = cv::RotatedRect(armor_singer_center,
                                                cv::Size2f(Track.enemy_armor.size.width,Track.enemy_armor.size.height),
                                                Track.enemy_armor.angle);

    cv::Point2f vertice_armor_singer[4];
    armor_singer.points(vertice_armor_singer);
    for (int m = 0; m < 4; ++m)
    {
        line(src, vertice_armor_singer[m], vertice_armor_singer[(m + 1) % 4], CV_RGB(255, 255, 0),2,cv::LINE_8);
    } 

    // Time
    ros::Time end = ros::Time::now();
    ros::Duration duration_t = end - begin;
    double delta_tt = duration_t.toSec();
    cv::putText(src,"FPS      : "+std::to_string(1/delta_tt),cv::Point2f(0,30),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    // ROS_INFO("FPS %lf \n", 1/delta_tt);

    cv::circle(src,cv::Point(640,512),5,cv::Scalar(255,200,100),-1);

    // int c = cv::waitKey(10);
    // std::string path0 = "/home/lmx2/data/buff0.jpg";
    // std::string path  = "/home/lmx2/data/buff.jpg";
    // if(c==113){
    //     cv::imwrite(path0,_src);
    //     cv::imwrite(path,src);
    // }

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
    aim_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("aim_point", 1);

    image_transport::ImageTransport it(nh);
    // image_transport::Subscriber camera_src_sub = it.subscribe("image_raw",1,ImageCallback);  //,image_transport::TransportHints("compressed")
    // ros::Subscriber camera_imu_sub = nh.subscribe("euler_angles",1,ImuCallback);
    // ros::Subscriber imu_sub = nh.subscribe("imu",1,imuCallback);

    message_filters::Subscriber<sensor_msgs::Image> camera_src_sub(nh, "image_raw", 1);  
    // // fdilink_ahrs
    // message_filters::Subscriber<robot_msgs::EulerAngles> camera_imu_sub(nh, "euler_angles", 1);  
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, robot_msgs::EulerAngles> MySyncPolicy;
    // c 板
    message_filters::Subscriber<robot_msgs::vision> camera_imu_sub(nh, "vision_data", 1);  
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, robot_msgs::vision> MySyncPolicy;
    // // // // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), camera_src_sub, camera_imu_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    return 0;
}
