#include <sstream>
#include <string>
#include "MVCameraCapture.h"

using namespace std; 

namespace cv_camera
{

namespace enc = sensor_msgs::image_encodings;

MVCameraCapture::MVCameraCapture(ros::NodeHandle &node, const std::string &topic_name,
                 int32_t buffer_size, const std::string &frame_id)
    : node_(node),
      it_(node_),
      topic_name_(topic_name),
      buffer_size_(buffer_size),
      frame_id_(frame_id)
{
}


void MVCameraCapture::open(int32_t device_id)
{
  int                     iCameraCounts = 1;
  int                     iStatus=-1;
  tSdkCameraDevInfo       tCameraEnumList;
  tSdkCameraCapbility     tCapability;      //设备描述信息
  int                     channel=3;

  // 0 for English, 1 for Chinese
  int status = CameraSdkInit(1);
  std::cout << "Camera SDK init status : " << status << std::endl;


  status = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
  if(iCameraCounts==0)
  {
    std::cout << "No camera found" << std::endl;
    std::cout << "Status = " << status << std::endl;

    std::stringstream stream;
    stream << "No camera found";
    throw DeviceError(stream.str());
  }

  iStatus = CameraInit(&tCameraEnumList,-1,-1, &hCamera_);
  std::cout << "hCamera = " << hCamera_ << std::endl;

  if(iStatus!=CAMERA_STATUS_SUCCESS)
  {
    std::stringstream stream;
    stream << "device_id" << device_id << " cannot be opened";
    throw DeviceError(stream.str());
  }

  CameraGetCapability(hCamera_,&tCapability);
  std::cout << "buffer size" <<  tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3 << std::endl;
  rgbBuffer_ = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);


  tSdkImageResolution psCurVideoSize;

//   CameraGetImageResolution(hCamera_, &psCurVideoSize);
//   psCurVideoSize.iWidthZoomHd = 1024;
//   psCurVideoSize.iHeightZoomHd = 1024;
//  CameraSetImageResolution(hCamera_, &psCurVideoSize);

  //CameraSetTriggerMode(hCamera_,2); // 自己加的程序，硬件触发才会有图片

// 手动曝光，曝光时间 2ms
    CameraSetAeState(hCamera_, FALSE);
    CameraSetExposureTime(hCamera_, 1200);
    // CameraSetWbMode(hCamera_,true);


    // 按 RGB 顺序提供颜色增益
    // CameraSetGain(hCamera_, 120, 110, 135);
    CameraSetAnalogGain(hCamera_, 152);  //该值增大后会提升图像背景噪声
    //    CameraSetContrast(h_camera, 200);
    //    CameraSetSaturation(h_camera, 1200);
    //    CameraSetSharpness(h_camera, 10);

  CameraPlay(hCamera_);

  if(tCapability.sIspCapacity.bMonoSensor)
  {
    channel=1;
    CameraSetIspOutFormat(hCamera_,CAMERA_MEDIA_TYPE_MONO8);
  }

  else
  {
    channel=3;
    CameraSetIspOutFormat(hCamera_,CAMERA_MEDIA_TYPE_BGR8);
  }

  pub_ = it_.advertise(topic_name_, buffer_size_);

  //loadCameraInfo();
}

void MVCameraCapture::open()
{
  open(0);
  // CameraSetAeState(hCamera_,FALSE);
}

bool MVCameraCapture::capture()
{
  uchar * pbyBuffer =0;
  if(CameraGetImageBuffer(hCamera_, &frameInfo_,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
  {
    CameraImageProcess(hCamera_, pbyBuffer, rgbBuffer_, &frameInfo_);
    //char *buffer = (char *)malloc(2048 * 2048 * 3);
    //memcpy(rgbBuffer_, buffer, 2048 * 2048 * 3);
    
   // 此处有问题，已经修改过来
   // bridge_.image = cv::Mat(frameInfo_.iHeight, frameInfo_.iWidth, CV_8UC3, rgbBuffer_, cv::Mat::AUTO_STEP);
    
    bridge_.image = cv::Mat(cvSize(frameInfo_.iWidth,frameInfo_.iHeight), frameInfo_.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,rgbBuffer_);
    
    
//--------------------------自己加的程序-保存图像到路径 ----------------------------------//
  //   double image_time = ros::Time::now().toSec(); 
  //   image_num ++;
    
  //   string image_name;
  //   //std::to_string(image_time) 数字 到 string 
  //   image_name = std::to_string(image_time) + ".jpg";      //图像命名：时间戳.jpg
    
  //   cv::Mat cv_img = bridge_.image;
  //  // cv::imwrite(cam0_path + image_name, cv_img);    //保存图片
                  
//--------------------------自己加的程序-保存图像到路径 ----------------------------------//    
    
    // std::cout << bridge_.image.rows << std::endl;
    // cv::resize(bridge_.image, bridge_.image, cv::Size(640, 480));
    // cv::imshow("iamge", bridge_.image);
    // cv::waitKey(5);
    

    ros::Time now = ros::Time::now();
    bridge_.encoding = enc::BGR8;
    bridge_.header.stamp = now;
    bridge_.header.frame_id = frame_id_;

    CameraReleaseImageBuffer(hCamera_, pbyBuffer);
    return true;
  }
  return false;
}

void MVCameraCapture::publish()
{
  pub_.publish(*getImageMsgPtr());
}



} // namespace cv_camera
