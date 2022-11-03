#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vision_msgs/Detection2DArray.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>

class Detection
{
  private:
    ros::NodeHandle nh_;
    ros::Subscriber camera_sub_;
    ros::Subscriber depth_sub_;
    ros::Publisher bbox_pub_;

    std::unique_ptr<Detector> detector_;
    
  public:
    Detection(ros::NodeHandle &nh);
    void detectionCallback(const sensor_msgs::Image &img);
    vision_msgs::Detection2DArray RectToDetection2DArray(const std::vector<cv::Rect> &bbox);
};
