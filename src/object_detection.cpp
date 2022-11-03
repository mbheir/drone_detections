#include "object_detection.hpp"
#include "nms.hpp"

// use 

Detection::Detection(ros::NodeHandle &nh)
  : nh_(nh)
{
  camera_sub_ = nh_.subscribe("/camera/color/image_raw", 1, &Detection::detectionCallback, this); // Double check topic
  bbox_pub_ = nh_.advertise<vision_msgs::Detection2DArray>("/detections", 1);

  std::string config_path = ""; // Path to yolo config file
  std::string weights_path = ""; // Path to yolo weights file 

  
}

void Detection::detectionCallback(const sensor_msgs::Image &img)
{
  // this callback runs every time a msg is received on the camera topic, performs some action and publish bbox on publisher'



  float threshold = 0.5;
  vision_msgs::Detection2DArray bbox;
  std::vector<std::vector<float>> bboxes = GetBBoxes(img); // GetBBoxes is a function that returns a vector of vectors of floats. Not implemented
  std::vector<cv::Rect> bbox_nms = nms(bboxes, threshold);
  vision_msgs::Detection2DArray bbox_msg = RectToDetection2D(bbox_nms);
  bbox_pub_.publish(bbox_msg);
}

vision_msgs::Detection2DArray Detection::RectToDetection2D(std::vector<cv::Rect> &bbox)
{
  vision_msgs::Detection2DArray bbox_msg;
  for (int i = 0; i < bbox.size(); i++)
  {
    vision_msgs::Detection2D detection;
    vision_msgs::BoundingBox2D bbox_i;
    
    bbox_i.bbox.center.x = bbox[i].x + bbox[i].width / 2;
    bbox_i.bbox.center.y = bbox[i].y + bbox[i].height / 2;
    bbox_i.bbox.size_x = bbox[i].width;
    bbox_i.bbox.size_y = bbox[i].height;
    detection.bbox = bbox_i;
    bbox_msg.detections.push_back(detection);
  }
  
  return bbox_msg;

}