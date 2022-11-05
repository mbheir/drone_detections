// Sends camera feed to yolo node
//  - Synchronization of camera feed and depth 

// Listen to the bounding box and depth topic. Find average depth in bbox
//  - Update bbox type to include depth

#include "detection.hpp"

Detection::Detection(ros::NodeHandle &nh)
: nh_(nh)
{
    camera_sub_ = nh_.subscribe("/camera/color/image_raw", 1, &Detection::cameraCallback, this); // Double check topic. Maybe unnecessary. 
    bbox_sub_ = nh_.subscribe("/yolo/detection", 1, &Detection::bboxCallback, this); // Find topic
    depth_sub_ = nh_.subscribe("/camera/depth", 1, &Detection::bboxCallback, this); // Check topic
    image_to_yolo_pub_ = nh.advertise<sensor_msgs::Image>("/yolo/image", 1); // Publish image to yolo
    bbox_depth_pub_ = nh_.advertise<vision_msgs::Detection2DArray>("/", 1); // Find 
}

void Detection::cameraCallback(const sensor_msgs::Image &img)
{
    last_image_received_ = img;
    image_received_ = true;
}

void Detection::depthCallback(const sensor_msgs::Image &depth_img)
{
    last_depth_image_received_ = depth_img;
    depth_image_received_ = true;
}

void Detection::bboxCallback(const vision_msgs::Detection2DArray &bboxes)
{
    std::vector<vision_msgs::Detection2D> detections = bboxes.detections;
    vision_msgs::Detection2DArray bboxes_with_depth;
    bboxes_with_depth.detections = detections;
    bboxes_with_depth.header = bboxes.header;

    float depth;
    for (int i = 0; i < detections.size(); i++) 
    {
        depth = calcualte_avg_depth(detections[i]);
        bboxes_with_depth.detections[i].bbox.center.theta = static_cast<double>(depth);
    }
     
    bbox_depth_pub_.publish(bboxes_with_depth);
}

void Detection::sync_and_broadcast_image()
{
    ros::Rate r(50);
    while (ros::ok())
    {
        ros::spinOnce();
        if (image_received_ && depth_image_received_) 
        {
            image_received_ = false;
            depth_image_received_ = false;

            // Check timestamps:
            ros::Time time_image = last_image_received_.header.stamp;
            ros::Time time_depth = last_depth_image_received_.header.stamp;
            if (time_image == time_depth)
            {
                image_to_yolo_pub_.publish(last_image_received_);
                depth_image_sent_ = last_depth_image_received_;
            }
        } 
        r.sleep();
    }
}

float Detection::calcualte_avg_depth(vision_msgs::Detection2D &bbox)
{
    // Find average depth in bbox
    // Update bbox type to include depth
    geometry_msgs::Pose2D center = bbox.bbox.center;
    float size_x = bbox.bbox.size_x;
    float size_y = bbox.bbox.size_y;
    float x0 = center.x - size_x/2;
    float y0 = center.y - size_y/2;

    float cum_depth = 0;

    for (int j = 0; j < size_x; j++)
    {
        for (int k = 0; k < size_y; k++)
        {
            cum_depth += depth_image_sent_.data[(y0+k)*(x0+j)];
        }
    }
    float avg_depth = cum_depth/(size_x*size_y);
    return avg_depth;
}