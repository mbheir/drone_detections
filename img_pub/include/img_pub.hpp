#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImgPub
{
  private:
    ros::NodeHandle nh_;
    ros::Publisher img_pub_;

  public:
    ImgPub(ros::NodeHandle &nh);
};