#include "img_pub.hpp"


ImgPub::ImgPub(ros::NodeHandle& nh)
    : nh_(nh)
{
    img_pub_ = nh_.advertise<sensor_msgs::Image>("/yolo/image", 1);

    cv::Mat img = cv::imread("/home/olewam/catkin_ws/src/drone_detections/img_pub/include/1864.jpg");

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

    img_pub_.publish(img_msg);
    ros::Rate loop_rate(0.1);

    while (nh.ok()) {
        img_pub_.publish(img_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

