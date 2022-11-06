#include "img_pub.hpp"


ImgPub::ImgPub(ros::NodeHandle& nh)
    : nh_(nh)
{
    img_pub_ = nh_.advertise<sensor_msgs::Image>("/yolo/image", 1);

    cv::Mat image = cv::imread("/home/olewam/detection_ws/src/drone_detections/img_pub/include/1864.jpg");

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image).toImageMsg();

    ros::Rate loop_rate(0.1);

    while (nh.ok()) {
        ROS_INFO("Publish image");
        img_pub_.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

