#include "detection.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection");
    ros::NodeHandle nh;
    Detection detection(nh);

    ros::spin();
}