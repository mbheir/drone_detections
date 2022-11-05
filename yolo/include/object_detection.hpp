#include "ros/ros.h"
#include "vision_msgs/Detection2DArray.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class ObjectDetection
{
  private:
    ros::NodeHandle nh_;
    ros::Subscriber camera_sub_;
    ros::Publisher bbox_pub_;

    cv::dnn::Net net_;


  public:
    ObjectDetection(ros::NodeHandle &nh);
    void ObjectDetectionCallback(const sensor_msgs::Image &img);
    // vision_msgs::Detection2DArray RectToDetection2DArray(const std::vector<cv::Rect> &bbox);
    std::vector<std::string> getOutputNames();
};
