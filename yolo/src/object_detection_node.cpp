#include "object_detection.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detections");
    ros::NodeHandle nh;
    ObjectDetection objectDetection(nh);

    ros::spin();
}