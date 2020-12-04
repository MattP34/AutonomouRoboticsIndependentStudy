#include <string>

//imporing libraies

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/string.hpp"

//Create Odometry to TF class to do the conversion
class OdometryToTF : public rclcpp::Node //extend node class
{
public:
  OdometryToTF(): Node("odemetry_to_tf") {
    //create a subssciper to the ros topic /model/X1/odometry
    odometrySubscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/model/X1/odometry",
      rclcpp::QoS(10), std::bind(&OdometryToTF::topic_callback, this, std::placeholders::_1));
  }

private:
  //what is called when subscriber gets new data
  void topic_callback(std::shared_ptr<const nav_msgs::msg::Odometry> msg) {
    //brocast a tf transformation from frame odom to frame X1/base_link/front_laser based on odomtry data
    tf2_ros::TransformBroadcaster br = tf2_ros::TransformBroadcaster(Node("test_tf"));
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp.sec = msg->header.stamp.sec;
    transformStamped.header.stamp.nanosec = msg->header.stamp.nanosec;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "X1/base_link";
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

    //physical transformation from center of wheel based to the lidar sensor
    geometry_msgs::msg::TransformStamped transformStamped2;
    transformStamped2.header.stamp.sec = msg->header.stamp.sec;
    transformStamped2.header.stamp.nanosec = msg->header.stamp.nanosec;
    transformStamped2.header.frame_id = "X1/base_link";
    transformStamped2.child_frame_id = "X1/base_link/front_laser";
    transformStamped2.transform.translation.x = 0.08-0.256;
    transformStamped2.transform.translation.y = 0;
    transformStamped2.transform.translation.z = 0.52628;
    transformStamped2.transform.rotation.x = 0;
    transformStamped2.transform.rotation.y = 0;
    transformStamped2.transform.rotation.z = 0;
    transformStamped2.transform.rotation.w = 1;

    //brocast a tf transformation from frame map to frame X1/base_link based on odomtry data (robot position)
    geometry_msgs::msg::TransformStamped transformStamped3;
    transformStamped3.header.stamp.sec = msg->header.stamp.sec;
    transformStamped3.header.stamp.nanosec = msg->header.stamp.nanosec;
    transformStamped3.header.frame_id = "map";
    transformStamped3.child_frame_id = "X1/base_link";
    transformStamped3.transform.translation.x = msg->pose.pose.position.x;
    transformStamped3.transform.translation.y = msg->pose.pose.position.y;
    transformStamped3.transform.translation.z = 0.0;
    transformStamped3.transform.rotation.x = msg->pose.pose.orientation.x;
    transformStamped3.transform.rotation.y = msg->pose.pose.orientation.y;
    transformStamped3.transform.rotation.z = msg->pose.pose.orientation.z;
    transformStamped3.transform.rotation.w = msg->pose.pose.orientation.w;

    //would comment which transform to send based on needs of the program (just the first for SLAM_Toolbox, last 2 for odomtry movement)
    br.sendTransform(transformStamped);
    br.sendTransform(transformStamped2);
    br.sendTransform(transformStamped3);
  }
  //class varible declaration
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odometrySubscription_;
};

//main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //run the node
  rclcpp::spin(std::make_shared<OdometryToTF>());
  rclcpp::shutdown();
  return 0;
}
