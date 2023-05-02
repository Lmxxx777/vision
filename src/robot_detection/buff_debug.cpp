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

using namespace cv;

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
    // buff_detection.AS.init(roll, pitch, yaw, quaternion, bullet_speed);

    // buff-detecting
    // buff_detection.detectResult(src,now_time);


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

// int main(int argc, char  *argv[])
// {
//     setlocale(LC_ALL,"");

// 	cv::VideoCapture cap;
// 	cap.open("/home/lmx2/rune_test.mp4"); 

// 	if (!cap.isOpened())
// 		return 0;

//     cv::Mat src;
//     robot_detection::BuffDetector buff_detection;

// 	cv::Mat frame;
// 	while (1)
// 	{
// 		cap >> frame; 
// 		if (frame.empty())
// 			break;
// 		// imshow("video", frame);

//         buff_detection._src = frame;

//         std::vector<cv::Mat> bgr;
//         cv::split(frame,bgr);
        
        
//         // buff_detection.setImage(frame);

//         // cv::cvtColor(bgr[0],bgr[0],cv::COLOR_BGR2GRAY);
//         // cv::threshold(bgr[0],buff_detection._binary,200,255,cv::THRESH_BINARY);

//         buff_detection._binary = bgr[0];
//         cv::threshold(buff_detection._binary,buff_detection._binary,200,255,cv::THRESH_BINARY);
//         buff_detection.extractContours();
//         // buff_detection.findRcenter();

//  imshow("b",buff_detection._binary);


// 		char c = cv::waitKey(50);
// 		if (c == 'q') {
// 			break;
// 		}
// 	}
// 	cap.release();
//     return 0;
// }

int main()
{
    // // 打开文件
	// VideoCapture capture;
	// capture.open("/home/lmx2/rune_test.mp4");
	// if (!capture.isOpened()) {
	// 	printf("could not read this video file...\n");
	// 	return -1;
	// }
	// Size S = Size((int)capture.get(CAP_PROP_FRAME_WIDTH),
	// 	(int)capture.get(CAP_PROP_FRAME_HEIGHT));
	// int fps = capture.get(CAP_PROP_FPS);
	// printf("current fps : %d \n", fps);
	// VideoWriter writer("/home/lmx2/rune_test_2.mp4", CAP_OPENCV_MJPEG, fps, S, true);

	// Mat frame;
	// namedWindow("camera-demo", WINDOW_AUTOSIZE);
	// while (capture.read(frame)) {
	// 	imshow("camera-demo", frame);
	// 	writer.write(frame);
	// 	char c = waitKey(50);
	// 	if (c == 27) {
	// 		break;
	// 	}
	// }
	// capture.release();
	// writer.release();

    cv::Mat buff = cv::imread("/home/lmx2/data/buff0.jpg");
    // cv::Mat buff = cv::imread("/home/lmx2/data/buff_r.jpg");
    // // save R r0i
    // cv::Rect rt = cv::Rect(cv::Point(560,500),cv::Point(600,540));
    // cv::Mat r_roi = buff(rt);
    // cv::imwrite("/home/lmx2/data/buff_r.jpg",r_roi);


    float quaternion[4] = {
        0.915764,
        0.049038,
        0.021370,
        0.398140,
    };
    // buff_detection.AS.init(0, 0, 0, quaternion, 28);
    // buff-detecting
    robot_detection::chrono_time now_time = std::chrono::high_resolution_clock::now();

    buff_detection.isInitYaw = true;
    buff_detection.detectResult(buff,now_time);

    // cv::Mat channels[3];
    // cv::split(buff, channels); // 分离多通道图像的通道
    // cv::imshow("b",channels[0]);
    // cv::imshow("g",channels[1]);
    // threshold(channels[1], channels[1], 0, 255, THRESH_OTSU );
    // cv::imshow("r",channels[1]);

    // 测试：识别R的距离
    // cv::Point2f points_4[4];
    // points_4[0] = cv::Point2f(572, 514);
    // points_4[1] = cv::Point2f(592, 514);
    // points_4[2] = cv::Point2f(592, 534);
    // points_4[3] = cv::Point2f(572, 534);
    // points_4[0] = cv::Point2f(582 - buff_detection.r_width_pixel/2, 524 - buff_detection.r_height_pixel/2);
    // points_4[1] = cv::Point2f(582 + buff_detection.r_width_pixel/2, 524 - buff_detection.r_height_pixel/2);
    // points_4[2] = cv::Point2f(582 + buff_detection.r_width_pixel/2, 524 + buff_detection.r_height_pixel/2);
    // points_4[3] = cv::Point2f(582 - buff_detection.r_width_pixel/2, 524 + buff_detection.r_height_pixel/2);
    // Eigen::Vector3d pos = buff_detection.AS.pixel2imu(points_4, robot_detection::BUFF_R); 
    // std::cout<<"distance: "<<pos.norm()<<std::endl;

    cv::imshow("result",buff);
	cv::waitKey(0);
	return 0;
}