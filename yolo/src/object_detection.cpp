#include "object_detection.hpp"


ObjectDetection::ObjectDetection(ros::NodeHandle &nh)
  : nh_(nh)
{
  camera_sub_ = nh_.subscribe("/camera/color/image_raw", 1, &ObjectDetection::ObjectDetectionCallback, this); // Double check topic
  bbox_pub_ = nh_.advertise<vision_msgs::Detection2DArray>("/yolo/bbox", 1);
  detection_pub_ = nh_.advertise<sensor_msgs::Image>("/yolo/detection", 1);


  std::string config_path;
  std::string weights_path;
  std::string names_path;
  nh_.getParam("/yolo/config_path", config_path);
  nh_.getParam("/yolo/weights_path", weights_path);
  nh_.getParam("/yolo/names_path", names_path);

  // Load network
  net_ = readNetFromDarknet(config_path, weights_path);
  net_.setPreferableBackend(DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(DNN_TARGET_CPU);

  ifstream f(names_path.c_str());
  string label;
  while (getline(f, label))
  {
    classes_.push_back(label);
  } 
}

void ObjectDetection::ObjectDetectionCallback(const sensor_msgs::ImagePtr &img)
{
  // this callback runs every time a msg is received on the camera topic, performs some action and publish bbox on publisher
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
  
  Mat blob;
  blobFromImage(cv_ptr->image, blob, 1/255.0, Size(416, 416), Scalar(0,0,0), true, false);
  net_.setInput(blob);

  // Run forward pass through the network to find detections
  vector<Mat> detections;
  net_.forward(detections, getOutputNames());
  ROS_INFO("Forward pass complete\n");

  for (size_t i = 0; i < detections.size(); i++)
  {
    float* detection_data = (float*) detections[i].data;
    for (int j = 0; j < detections[i].rows; j++, detection_data += detections[i].cols)
    {
      Mat scores = detections[i].row(j).colRange(5, detections[i].cols);
      Point classIdPoint;
      double confidence;
      minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
      if (confidence > confidence_threshold_)
      {
        ROS_INFO_STREAM("Detection with confidence: " << confidence << "\n");
        int centerX = (int)(detection_data[0] * (cv_ptr->image).cols);
        int centerY = (int)(detection_data[1] * (cv_ptr->image).rows);
        int width = (int)(detection_data[2] * (cv_ptr->image).cols);
        int height = (int)(detection_data[3] * (cv_ptr->image).rows);
        // int centerX = (int)(detection_data[0] * 720);
        // int centerY = (int)(detection_data[1] * 540);
        // int width = (int)(detection_data[2] * 720);
        // int height = (int)(detection_data[3] * 540);
        int bbox_top_left_x = centerX - width / 2;
        int bbox_top_left_y = centerY - height / 2;

        classIds_.push_back(classIdPoint.x);
        confidences_.push_back((float)confidence);
        boxes_.push_back(Rect(bbox_top_left_x, bbox_top_left_y, width, height));
      }
    }

    // Perform non maximum suppression to remove bboxes that overlap
    vector<int> indices;
    NMSBoxes(boxes_, confidences_, confidence_threshold_, nms_threshold_, indices);
    
    // Send bbox detections here or in the loop
    vision_msgs::Detection2DArray bbox_array;
    for (auto i : indices)
    {
      Rect box = boxes_[i];

      vision_msgs::Detection2D bbox;
      geometry_msgs::Pose2D center;

      bbox.header.stamp = img->header.stamp;
      // bbox.source_img = *img;
      center.x = box.x + box.width / 2; // Center x is top left x + half of the width
      center.y = box.y + box.height / 2; // Center y is top left y + half of the height
      bbox.bbox.center = center;
      bbox.bbox.size_x = box.width;
      bbox.bbox.size_y = box.height;
      bbox_array.header.stamp = img->header.stamp;
      bbox_array.detections.push_back(bbox);

      // Draw bounding box for debug purposes
      drawPred(classIds_[i], confidences_[i], box.x, box.y, box.x + box.width, box.y + box.height, cv_ptr->image);

    }
    bbox_pub_.publish(bbox_array);
    sensor_msgs::ImagePtr detection_frame = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cv_ptr->image).toImageMsg();
    detection_pub_.publish(detection_frame);
  }
      
  // imwrite("/home/olewam/detection_ws/src/drone_detections/img_pub/include/pub_1864_bbox.jpg", cv_ptr->image);  

}

void ObjectDetection::drawPred(int classId, float conf, int top_left_x, int top_left_y, int width, int height, Mat& frame)
{
  /*
    Based on https://learnopencv.com/deep-learning-based-object-detection-using-yolov3-with-opencv-python-c/
  */

  //Draw a rectangle displaying the bounding box
  rectangle(frame, Point(top_left_x, top_left_y), Point(width, height), Scalar(255, 178, 50), 3);
    
  //Get the label for the class name and its confidence
  string label = format("%.2f", conf);
  if (!classes_.empty())
  {
      CV_Assert(classId < (int)classes_.size());
      label = classes_[classId] + ":" + label;
  }
    
  //Display the label at the top of the bounding box
  int baseLine;
  Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
  top_left_y = max(top_left_y, labelSize.height);
  rectangle(frame, Point(top_left_x, top_left_y - round(1.5*labelSize.height)), Point(top_left_x + round(1.5*labelSize.width), top_left_y + baseLine), Scalar(255, 255, 255), FILLED);
  putText(frame, label, Point(top_left_x, top_left_y), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,0),1);
}

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