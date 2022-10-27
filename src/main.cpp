#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vision_msgs/Detection2DArray.h"
#include "sensor_msgs/Image.h"

#include <sstream>

// use 

class Detection
{
  private:
    ros::NodeHandle nh_;
    ros::Subscriber camera_sub_;
    ros::Subscriber depth_sub_;
    ros::Publisher bbox_pub_;
  public:
    Detection(ros::NodeHandle &nh);
    void detectionCallback(const sensor_msgs::Image &img);
}

Detection::Detection(ros::NodeHandle &nh)
  : nh_(nh)
{
  camera_sub_ = nh_.subscribe("/camera/color/image_raw", 1, &Detection::detectionCallback); // Double check topic
  bbox_pub_ = nh_.advertise<vision_msgs::Detection2DArray>("/detections", 1);
}

void Detection::detectionCallback(const sensor_msgs::Image &img)
{
  // this callback runs every time a msg is received on the camera topic, performs some action and publish bbox on publisher

  vision_msgs::Detection2DArray bbox;

  //bbox_pub_.publish(bbox);
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "drone_detections");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;
  Detection(nh);

  ros::spin();

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */


  // ros::Rate loop_rate(10);

  // /**
  //  * A count of how many messages we have sent. This is used to create
  //  * a unique string for each message.
  //  */
  // int count = 0;
  // while (ros::ok())
  // {
  //   /**
  //    * This is a message object. You stuff it with data, and then publish it.
  //    */
  //   std_msgs::String msg;

  //   std::stringstream ss;
  //   ss << "hello world " << count;
  //   msg.data = ss.str();

  //   ROS_INFO("%s", msg.data.c_str());

  //   /**
  //    * The publish() function is how you send messages. The parameter
  //    * is the message object. The type of this object must agree with the type
  //    * given as a template parameter to the advertise<>() call, as was done
  //    * in the constructor above.
  //    */
  //   chatter_pub.publish(msg);

  //   ros::spinOnce();

  //   loop_rate.sleep();
  //   ++count;
  // }


  // return 0;
}