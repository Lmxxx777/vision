// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cv_camera");
  ros::NodeHandle private_node("~");
  cv_camera::Driver driver(private_node, private_node);



  // Time
  ros::Time begin;
  ros::Time end;

  try
  {
    driver.setup();
    while (ros::ok())
    {
      // Time
      end = ros::Time::now();
      ros::Duration duration = end - begin;
      double delta_tt = duration.toSec();
      ROS_INFO("FPS %lf \n", 1/delta_tt);
      begin = end;

      driver.proceed();
      ros::spinOnce();
    }
  }
  catch (cv_camera::DeviceError &e)
  {
    ROS_ERROR_STREAM("cv camera open failed: " << e.what());
    return 1;
  }

  return 0;
}
