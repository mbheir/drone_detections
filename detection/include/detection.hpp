#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vision_msgs/Detection2DArray.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>

class Detection 
{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber camera_sub_; // Maybe unnecessary. Wait with implementation
        ros::Subscriber bbox_sub_;
        ros::Subscriber depth_sub_;
        ros::Publisher image_to_yolo_pub_;
        ros::Publisher bbox_depth_pub_;

        sensor_msgs::Image last_image_received_;
        std::vector<sensor_msgs::Image> last_depth_image_received_;
        sensor_msgs::Image depth_image_sent_;

        ros::Time sync_threshold_;

        bool image_received_ = false;
        bool depth_image_received_ = false;
        // void sync_and_broadcast_image();
        float calculate_avg_depth(vision_msgs::Detection2D &bbox, const ros::Time stamp);
    
    public:
        Detection(ros::NodeHandle &nh);
        // void cameraCallback(const sensor_msgs::Image &img);  
        void bboxCallback(const vision_msgs::Detection2DArray &bboxes);
        void depthCallback(const sensor_msgs::Image &depth_img);
};

