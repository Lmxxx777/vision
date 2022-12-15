// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "driver.h"
#include <string>



namespace
{
const double DEFAULT_RATE = 180.0; // 30
const int32_t PUBLISHER_BUFFER_SIZE = 1;
}


boost::shared_ptr<cv_camera::MVCameraCapture> sCamera;


namespace cv_camera
{

Driver::Driver(ros::NodeHandle &private_node, ros::NodeHandle &camera_node)
    : private_node_(private_node),
      camera_node_(camera_node)
{
}

void Driver::setup()
{
  double hz(DEFAULT_RATE);
  int32_t device_id(0);
  std::string device_path("");
  std::string frame_id("camera");
  std::string file_path("");

  private_node_.getParam("device_id", device_id);
  private_node_.getParam("frame_id", frame_id);
  private_node_.getParam("rate", hz);

  int32_t image_width(1280);
  int32_t image_height(1024);

  camera_.reset(new MVCameraCapture(camera_node_,
                            "image_raw",
                            PUBLISHER_BUFFER_SIZE,
                            frame_id));

  sCamera = camera_;
  std::cout << "openin the camera device id \n";
  camera_->open(device_id);
  
  rate_.reset(new ros::Rate(hz));
}


void Driver::proceed()
{
  if (camera_->capture())
  {
    camera_->publish();
  }
  rate_->sleep();
}

Driver::~Driver()
{
}

} // namespace cv_camera
