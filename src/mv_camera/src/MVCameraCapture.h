#ifndef MVCAMERACAPTURE_H
#define MVCAMERACAPTURE_H

#include "exception.h"
#include <string>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/highgui/highgui.hpp>
#include <camera_info_manager/camera_info_manager.h>

#include <CameraApi.h>


// Fre  30  


/**
 * @brief namespace of this package
 */
namespace cv_camera
{

/**
 * @brief captures by cv::VideoCapture and publishes to ROS topic.
 *
 */
class MVCameraCapture
{
public:
  /**
   * @brief costruct with ros node and topic settings
   *
   * @param node ROS node handle for advertise topic.
   * @param topic_name name of topic to publish (this may be image_raw).
   * @param buffer_size size of publisher buffer.
   * @param frame_id frame_id of publishing messages.
   */
  MVCameraCapture(ros::NodeHandle &node,
          const std::string &topic_name,
          int32_t buffer_size,
          const std::string &frame_id);

  /**
   * @brief Open capture device with device ID.
   *
   * @param device_id id of camera device (number from 0)
   * @throw cv_camera::DeviceError device open failed
   *
   */
  void open(int32_t device_id);
  
  /**
   * @brief Open default camera device.
   *
   * This opens with device 0.
   *
   * @throw cv_camera::DeviceError device open failed
   */
  void open();

  /**
   * @brief capture an image and store.
   *
   * to publish the captured image, call publish();
   * @return true if success to capture, false if not captured.
   */
  bool capture();

  /**
   * @brief Publish the image that is already captured by capture().
   *
   */
  void publish();

  /**
   * @brief accessor of cv::Mat
   *
   * you have to call capture() before call this.
   *
   * @return captured cv::Mat
   */
  inline const cv::Mat &getCvImage() const
  {
    return bridge_.image;
  }

  /**
   * @brief accessor of ROS Image message.
   *
   * you have to call capture() before call this.
   *
   * @return message pointer.
   */
  inline const sensor_msgs::ImagePtr getImageMsgPtr() const
  {
    // return bridge_.toCompressedImageMsg(cv_bridge::JPG);
    return bridge_.toImageMsg();
  }

private:

  /**
   * @brief node handle for advertise.
   */
  ros::NodeHandle node_;

  /**
   * @brief ROS image transport utility.
   */
  image_transport::ImageTransport it_;

  /**
   * @brief name of topic without namespace (usually "image_raw").
   */
  std::string topic_name_;

  /**
   * @brief header.frame_id for publishing images.
   */
  std::string frame_id_;
  /**
   * @brief size of publisher buffer
   */
  int32_t buffer_size_;

  /**
   * @brief image publisher created by image_transport::ImageTransport.
   */
  image_transport::Publisher pub_;

  /**
   * @brief this stores last captured image.
   */
  cv_bridge::CvImage bridge_;

  unsigned char *rgbBuffer_;

  bool autoExposureOn_ = false;
  bool autoGainOn_ = false;

  /**
   * @brief MVCamera Info
   */

  int hCamera_;
  tSdkFrameHead frameInfo_;

};

} // namespace cv_camera

#endif // CV_CAMERA_CAPTURE_H
