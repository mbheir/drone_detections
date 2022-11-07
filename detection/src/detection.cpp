// Sends camera feed to yolo node
//  - Synchronization of camera feed and depth 

// Listen to the bounding box and depth topic. Find average depth in bbox
//  - Update bbox type to include depth

#include "detection.hpp"

Detection::Detection(ros::NodeHandle &nh)
: nh_(nh)
{
    // camera_sub_ = nh_.subscribe("/camera/color/image_raw", 1, &Detection::cameraCallback, this); // Double check topic. Maybe unnecessary. 
    bbox_sub_ = nh_.subscribe("/yolo/bbox", 1, &Detection::bboxCallback, this); // Find topic
    depth_sub_ = nh_.subscribe("/camera/depth", 1, &Detection::bboxCallback, this); // Check topic
    // image_to_yolo_pub_ = nh.advertise<sensor_msgs::Image>("/yolo/image", 1); // Publish image to yolo
    bbox_depth_pub_ = nh_.advertise<vision_msgs::Detection2DArray>("/detection", 1); // Find 
}

void Detection::cameraCallback(const sensor_msgs::Image &img)
{
    last_image_received_ = img;
    image_received_ = true;
    sync_and_broadcast_image()

}

void Detection::depthCallback(const sensor_msgs::Image &depth_img)
{
    last_depth_image_received_->push_back(depth_img);
    ROS_INFO_STREAM("Depth vec size pre loop: " << last_depth_image_received_->size() << "\n");
    for (auto di : last_dept_image_received_)
    {
        ROS_INFO_STREAM("Time diff: " << di->header.stamp - ros::Time::now() << ", result: " << di->header.stamp - ros::Time::now() < ros::Time(1) << "\n");
        if (di->header.stamp - ros::Time::now() < ros::Time(1))
        {
            last_depth_image_received_->remove(*di);
        }
    }
    ROS_INFO_STREAM("Depth vec size post loop: " << last_depth_image_received_->size() << "\n");
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
        depth = calculate_avg_depth(detections[i], detections[i].header.stamp);
        bboxes_with_depth.detections[i].bbox.center.theta = static_cast<double>(depth); // store depth in theta
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
            if ((time_image - time_depth) < sync_threshold_ && time_image.isValid()) // Timestamps within an acceptable thresholds and nonzero time
            {
                image_to_yolo_pub_.publish(last_image_received_);
                depth_image_sent_ = last_depth_image_received_;
            }
        } 
        r.sleep();
    }
}

float Detection::calculate_avg_depth(vision_msgs::Detection2D &bbox, const ros::Time stamp)
{
    // Find cosest depth data by time
    sensor_msgs::Image depth_image;
    depth_image.header.stamp = ros::Time(100000000);
    for (auto di : last_dept_image_received_)
    {
        if (std::abs(di->header.stamp - stamp) < std::abs(depth_image.header.stamp - stamp))
        {
            depth_image = *di;
        }
    }
    ROS_INFO_STREAM("Stamp: " << stamp << " and depth stamp: " << depth_image.header.stamp << "\n");

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
            cum_depth += depth_image.data[(y0+k)*(x0+j)];
        }
    }
    float avg_depth = cum_depth/(size_x*size_y);
    return avg_depth;
}