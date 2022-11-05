#include "img_pub.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "img_pub");
    ros::NodeHandle nh;
    ImgPub imgPub(nh);

    ros::spin();
}