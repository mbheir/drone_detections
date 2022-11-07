#include "ros/ros.h"
#include "vision_msgs/Detection2DArray.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;
using namespace dnn;

class ObjectDetection
{
  private:
    ros::NodeHandle nh_;
    ros::Subscriber camera_sub_;
    ros::Publisher bbox_pub_;
    ros::Publisher detection_pub_;

    Net net_;
    vector<string> classes_;
    vector<int> classIds_;
    vector<float> confidences_;
    vector<Rect> boxes_;
    float confidence_threshold_ = 0.5;
    float nms_threshold_ = 0.4;
    // std::unique_ptr<Detector> detector_;
    // std::vector<std::string> object_names_; 


  public:
    ObjectDetection(ros::NodeHandle &nh);
    void ObjectDetectionCallback(const sensor_msgs::ImagePtr &img);
    // vision_msgs::Detection2DArray RectToDetection2DArray(const std::vector<cv::Rect> &bbox);
    std::vector<std::string> getOutputNames();
    void drawPred(int classId, float conf, int top_left_x, int top_left_y, int width, int height, Mat& frame);
};
