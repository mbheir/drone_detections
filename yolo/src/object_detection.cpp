#include "object_detection.hpp"

ObjectDetection::ObjectDetection(ros::NodeHandle &nh)
  : nh_(nh)
{
  camera_sub_ = nh_.subscribe("/yolo/image", 1, &ObjectDetection::ObjectDetectionCallback, this); // Double check topic
  bbox_pub_ = nh_.advertise<vision_msgs::Detection2DArray>("/yolo/detections", 1);

  std::string config_path = "/home/olewam/catkin_ws/src/drone_detections/yolo/include/darpa_v3.cfg"; // Path to yolo config file 
  std::string weights_path = "/home/olewam/catkin_ws/src/drone_detections/yolo/include/darpa_v3_16000.weights"; // Path to yolo weights file 

  net_ = cv::dnn::readNetFromDarknet(config_path, weights_path);
}

void ObjectDetection::ObjectDetectionCallback(const sensor_msgs::Image &img)
{
  // this callback runs every time a msg is received on the camera topic, performs some action and publish bbox on publisher'
  // auto cv_img_rgb = std::make_shared<cv::Mat>(img.height, img.width, CV_8UC3, (void*)img.data.data());
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  cv::Mat blob;
  cv::dnn::blobFromImage(cv_ptr->image, blob, 1/255.0, cv::Size(416, 416), cv::Scalar(0,0,0), true, false);
  net_.setInput(blob);
  std::vector<cv::Mat> output;
  // std::vector<std::string> output_names;
  net_.forward(output, getOutputNames());
  // output = net_.forward();

  float threshold = 0.5; // For non-maximum suppression
  vision_msgs::Detection2DArray bbox_array;
  // std::vector<cv::Rect> bboxs;
  for (auto o : output)
  {
    ROS_INFO_STREAM("output 1: " << o.data[0]);
    ROS_INFO_STREAM("output 2: " << o.data[1]);
    ROS_INFO_STREAM("output 3: " << o.data[2]);
    ROS_INFO_STREAM("output 4: " << o.data[3]);
    vision_msgs::Detection2D bbox;
    float* data = (float*) o.data;
    int centerX = (int) (data[0] * cv_ptr->image.cols);
    int centerY = (int) (data[1] * cv_ptr->image.rows);
    int width = (int) (data[2] * cv_ptr->image.cols);
    int height = (int) (data[3] * cv_ptr->image.rows);
    // int top_x = centerX - width / 2;
    // int top_y = centerY - height / 2;
    // bboxs.push_back(cv::Rect(top_x, top_y, width, height));
    bbox.header.stamp = img.header.stamp;
    bbox.source_img = img;
    geometry_msgs::Pose2D center;
    center.x = centerX;
    center.y = centerY;
    bbox.bbox.center = center;
    bbox.bbox.size_x = width;
    bbox.bbox.size_y = height;
    bbox_array.header.stamp = img.header.stamp;
    bbox_array.detections.push_back(bbox);


    cv::rectangle(cv_ptr->image, cv::Point(bbox.bbox.center.x - bbox.bbox.size_x / 2, bbox.bbox.center.y - bbox.bbox.size_y / 2), cv::Point(bbox.bbox.center.x + bbox.bbox.size_x / 2, bbox.bbox.center.y + bbox.bbox.size_y / 2), cv::Scalar(255, 178, 50), 3);
    cv::imwrite("/home/olewam/catkin_ws/src/drone_detections/img_pub/include/1864_bbox.jpg", cv_ptr->image);
  }

  // std::vector<std::vector<float>> bboxes = GetBBoxes(img); // GetBBoxes is a function that returns a vector of vectors of floats. Not implemented
  // std::vector<cv::Rect> bbox_nms = nms(bboxes, threshold);
  // vision_msgs::Detection2DArray bbox_msg = RectToDetection2D(bbox_nms);
  bbox_pub_.publish(bbox_array);
}

// vision_msgs::Detection2DArray ObjectDetection::RectToDetection2DArray(const std::vector<cv::Rect> &bbox)
// {
//   vision_msgs::Detection2DArray bbox_msg;
//   for (int i = 0; i < bbox.size(); i++)
//   {
//     vision_msgs::Detection2D detection;
//     vision_msgs::BoundingBox2D bbox_i;
    
//     bbox_i.bbox.center.x = bbox[i].x + bbox[i].width / 2;
//     bbox_i.bbox.center.y = bbox[i].y + bbox[i].height / 2;
//     bbox_i.bbox.size_x = bbox[i].width;
//     bbox_i.bbox.size_y = bbox[i].height;
//     detection.bbox = bbox_i;
//     bbox_msg.detections.push_back(detection);
//   }
  
//   return bbox_msg;

// }

std::vector<std::string> ObjectDetection::getOutputNames()
{
    /*
      Based on example in https://learnopencv.com/deep-learning-based-object-detection-using-yolov3-with-opencv-python-c/
    */
    static std::vector<std::string> names;
    if (names.empty())
    {
        std::vector<int> outLayers = net_.getUnconnectedOutLayers();
        std::vector<std::string> layersNames = net_.getLayerNames();
         
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
        {
          names[i] = layersNames[outLayers[i] - 1];
        }
    }

    for (auto n : names)
    {
      ROS_INFO_STREAM("Name: " << n);
    }
    return names;
}