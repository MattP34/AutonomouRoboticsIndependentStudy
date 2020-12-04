#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

//library imports
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/string.hpp"

//class to determine and publish odometry tranformation from lidar data
class OdometryFromLidar : public rclcpp::Node //extend node class
{
public:
  OdometryFromLidar(): Node("odometry_from_lidar") { //create node called odometry_from_lidar
      //create subsriber to /lidar/points topic
      pointCloudSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar/points",
      rclcpp::QoS(10), std::bind(&OdometryFromLidar::topic_callback, this, std::placeholders::_1));
      //create publisher to publish the combined point cloud
      pointCloudPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("total_point_cloud", rclcpp::QoS(10));
      //setting values of class varibles
      lastPoints = sensor_msgs::msg::PointCloud();
      allPoints = sensor_msgs::msg::PointCloud();
      allPoints.header.set__frame_id("X1/base_link/front_laser");
      x_pos = 0;
      y_pos = 0;
      theta = 0;
  }

private:
    //what is called when subscriber gets new data
    void topic_callback(std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg) {
        RCLCPP_INFO(this->get_logger(), "published: 1 ", "test"); //print statement for debugging
        //covert PointCloud2 to PointCloud data type (easier to extract point data from)
        sensor_msgs::msg::PointCloud p1 = sensor_msgs::msg::PointCloud();
        sensor_msgs::convertPointCloud2ToPointCloud(*msg,p1);
        //delete points with position infinity
        for(int i = 0; i < p1.points.size(); i++) {
            if(isinf(p1.points[i].x) || isinf(p1.points[i].y)) {
                p1.points.erase(p1.points.begin() + i);
                i--;            
            }
        }
        //declarations for the first time running
        if(lastPoints.points.size()==0) {
            lastPoints = p1;
            allPoints = p1;
            return;
        }
        //print recieved points (for debugging)
        for(int i = 0; i < p1.points.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "point:" + std::to_string(i)+" x:"+std::to_string(p1.points[i].x)+" y:"+std::to_string(p1.points[i].y), "test2");
        }
        //print the previous received point cloud (for debugging)?
        for(int i = 0; i < lastPoints.points.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "lastPoints:" + std::to_string(i)+" x:"+std::to_string(lastPoints.points[i].x)+" y:"+std::to_string(lastPoints.points[i].y), "test2");
        }
        //gets points
        std::vector<geometry_msgs::msg::Point32> points = p1.points;
        //define varible vcalues
        double x_trans = 0.0, y_trans= 0.0, rotation = 0.0;
        double x_partial = 10; //doesn't really matter as long as while is true
        double y_partial = 10;
        double theta_partial = 10;
        double learning_rate = 0.01;//(last_time-(double)(msg->header.stamp.nanosec)/1000000000.0)*0.1; //tune
        double last_offset = 1000000000; //big_number
        double curr_offset = -5;
        //keep doing gradient decent until it stopped moving based an partial derivatives based on distance to previous points
        while(abs(x_partial)+abs(y_partial)+abs(theta_partial) > 0.0001) { //tune value
            if(curr_offset < -1) {
                //first time determine the last_offset from curr offset
                last_offset = determine_offset(transformPointCloudCopy(p1,x_pos,y_pos,theta),lastPoints);
            } else{
                last_offset = curr_offset;
            }
            //determining partial derivatives by slighlty shifting the point cloud and calculating the slope
            x_partial = (determine_offset(transformPointCloudCopy(p1,x_pos+x_trans+0.00001,y_pos+y_trans,theta+rotation),lastPoints)-last_offset)/0.00001; //tune
            y_partial = (determine_offset(transformPointCloudCopy(p1,x_pos+x_trans,y_pos+y_trans+0.00001,theta+rotation),lastPoints)-last_offset)/0.00001; //tune
            theta_partial = (determine_offset(transformPointCloudCopy(p1,x_pos+x_trans,y_pos+y_trans,theta+rotation+0.00001),lastPoints)-last_offset)/0.00001; //tune
            curr_offset = determine_offset(transformPointCloudCopy(p1,x_pos+x_trans-x_partial*learning_rate,y_pos+y_trans-y_partial*learning_rate,theta+rotation-theta_partial*learning_rate),lastPoints);
            //end loop if we did worse than last check
            if(curr_offset > last_offset) {
                break;
            }
            //translate by gradient decent (move based on partial derivatives and constant learning rate)
            x_trans+=x_partial*learning_rate*-1;
            y_trans+=y_partial*learning_rate*-1;
            rotation+=theta_partial*learning_rate*-1;
        }
        this->x_pos += x_trans;
        this->y_pos += y_trans;
        this->theta += rotation;
        x_trans = 0.0;
        y_trans= 0.0;
        rotation = 0.0;
        x_partial = 10; //doesn't really matter as long as while is true
        y_partial = 10;
        theta_partial = 10;
        learning_rate = 0.01;//(double)(msg->header.stamp.nanosec)/1000000000.0*0.1; //tune
        last_offset = 1000000000; //big_number
        curr_offset = -5;
        //same thing as above, but is determining offset based on all_points and not the last point cloud
        //this is to avoid small errors between two readings to add up and cause drift over time
        while(abs(x_partial)+abs(y_partial)+abs(theta_partial) > 0.0001) { //tune
            if(curr_offset < -1) {
                last_offset = determine_offset(transformPointCloudCopy(p1,x_pos,y_pos,theta),allPoints);
            } else{
                last_offset = curr_offset;
            }
            x_partial = (determine_offset(transformPointCloudCopy(p1,x_pos+x_trans+0.00001,y_pos+y_trans,theta+rotation),allPoints)-last_offset)/0.00001; //tune
            y_partial = (determine_offset(transformPointCloudCopy(p1,x_pos+x_trans,y_pos+y_trans+0.00001,theta+rotation),allPoints)-last_offset)/0.00001; //tune
            theta_partial = (determine_offset(transformPointCloudCopy(p1,x_pos+x_trans,y_pos+y_trans,theta+rotation+0.000001),allPoints)-last_offset)/0.000001; //tune
            curr_offset = determine_offset(transformPointCloudCopy(p1,x_pos+x_trans-x_partial*learning_rate,y_pos+y_trans-y_partial*learning_rate,theta+rotation-theta_partial*learning_rate),allPoints);
            if(curr_offset > last_offset) {
                break;
            }
            x_trans+=x_partial*learning_rate*-1;
            y_trans+=y_partial*learning_rate*-1;
            rotation+=theta_partial*learning_rate*-1;
        }
        this->x_pos += x_trans;
        this->y_pos += y_trans;
        this->theta += rotation;
        //create Quaternion to determine rotation
        tf2::Quaternion q;
        //define quaternion based on yaw rotation
        q.setRPY(0,0,rotation);
        //publish the translation data to tf2 from frame map to the lidar scan frame
        tf2_ros::TransformBroadcaster br = tf2_ros::TransformBroadcaster(Node("test_tf"));
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp.sec = msg->header.stamp.sec;
        transformStamped.header.stamp.nanosec = msg->header.stamp.nanosec;
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = msg->header.frame_id;
        transformStamped.transform.translation.x = this->x_pos;
        transformStamped.transform.translation.y = this->y_pos;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = q.getX();
        transformStamped.transform.rotation.y = q.getY();
        transformStamped.transform.rotation.z = q.getZ();
        transformStamped.transform.rotation.w = q.getW();
        br.sendTransform(transformStamped);
        //add the current point cloud to the total point cloud
        add_pointcloud(transformPointCloudCopy(p1,x_pos,y_pos,theta));
        //set last reading as current reading
        transformPointCloud(p1,x_pos,y_pos,theta);
        lastPoints = p1;
        //publish combined point cloud topic
        sensor_msgs::msg::PointCloud exportCloud = transformPointCloudCopy(allPoints,x_pos*-1,y_pos*-1,theta*-1);
        exportCloud.header.set__stamp(msg->header.stamp);
        exportCloud.header.set__frame_id("X1/base_link/front_laser");
        pointCloudPublisher_->publish(exportCloud);
    }
    //sort alorithm used in binary search
    static bool sortX(geometry_msgs::msg::Point32 p1, geometry_msgs::msg::Point32 p2) {
        return p1.x<p2.x;
    }
    //find the closest correspongding point in another point cloud
    int closest_point(geometry_msgs::msg::Point32 point, std::vector<geometry_msgs::msg::Point32> xsort2) {
        int minIndex = 0;
        int maxIndex = xsort2.size()-1;
        int index = 0;
        double residual = 10000000.0; //big number
        int returnIndex = 0;
        //binary search points on the x axis
        while(minIndex != maxIndex) {
            index = (maxIndex+minIndex)/2;
            if(point.x == xsort2[index].x) {
                break;
            } 
            if(maxIndex-minIndex==1) {
                if(point.x < xsort2[minIndex].x) {
                    index = minIndex;
                    break;
                }
                if(point.x > xsort2[maxIndex].x) {
                    index = maxIndex;
                    break;
                }
                if(point.x-xsort2[minIndex].x < xsort2[maxIndex].x-point.x) {
                    index = minIndex;
                    break;
                }
                index = maxIndex;
                break;
            }
            if(point.x < xsort2[index].x) {
                maxIndex = index;
            } else {
                minIndex = index;
            }
        }
        returnIndex = index;
        double tempr;
        //check distance to the points to the left and right of the original point to find the closest point
        //only goes as far as the closest found distance to avoid searching unnecessary points
        //left search
        for(int i = index; i >= 0 && abs(point.x-xsort2[i].x) < residual; i--) {
            tempr = sqrt(pow(point.x-xsort2[i].x,2)+pow(point.y-xsort2[i].y,2));
            if(tempr < residual) {
                residual = tempr;
                returnIndex = i;
            }
        }
        //right search
        for(int i = index; i < (int)xsort2.size() && pow(point.x-xsort2[i].x,2) < residual; i++) {
            tempr = sqrt(pow(point.x-xsort2[i].x,2)+pow(point.y-xsort2[i].y,2));
            if(tempr < residual) {
                residual = tempr;
                returnIndex = i;
            }
        }
        return returnIndex;
    }

    //determine the sum of average absolute residual between each point in v1 and the closest point in xsort2
    double determine_offset_helper(std::vector<geometry_msgs::msg::Point32> v1, std::vector<geometry_msgs::msg::Point32> xsort2) {
        double residualSum = 0.0;
        for(int i = 0; i < (int)v1.size(); i++) {
            int index = closest_point(v1[i],xsort2);
            residualSum += sqrt(pow(v1[i].x-xsort2[index].x,2)+pow(v1[i].y-xsort2[index].y,2));
        }
        return residualSum/v1.size();
    }

    //add a point cloud to the allPoints point cloud
    //also eliminates extranous points that are in relatively close proximaty
    void add_pointcloud(sensor_msgs::msg::PointCloud pc) {
        std::vector<geometry_msgs::msg::Point32> v1 = pc.points;
        for(int i = 0; i < (int)v1.size(); i++) {
            //find closest point to the current point
            int index = closest_point(v1[i],allPoints.points);
            //if the point is closer than 5mm delelte the extranous point
            if(sqrt(pow(v1[i].x-allPoints.points[index].x,2)+pow(v1[i].y-allPoints.points[index].y,2)) < 0.005) { //change value
                allPoints.points[index].set__x(v1[i].x);
                allPoints.points[index].set__y(v1[i].y);
            } else {
                //add the point to the allPoints cloud
                geometry_msgs::msg::Point32 tempPoint = geometry_msgs::msg::Point32();
                tempPoint.set__x(v1[i].x);
                tempPoint.set__y(v1[i].y);
                allPoints.points.push_back(tempPoint);
            }
        }
    }

    //determine the average distance between points in pc1 and their closest neighbor in pc2
    double determine_offset(sensor_msgs::msg::PointCloud  pc1, sensor_msgs::msg::PointCloud pc2) {
        //get the vector of points from the pointclouds
        std::vector<geometry_msgs::msg::Point32> v1 = pc1.points;
        std::vector<geometry_msgs::msg::Point32> v2 = pc2.points;
        //create a sorted vector of points in pc2
        std::vector<geometry_msgs::msg::Point32> xsort2 = std::vector<geometry_msgs::msg::Point32>(v2.size());
        for(int i = 0; i < (int)xsort2.size(); i++) {
            xsort2[i] = v2[i];
        }
        std::sort(xsort2.begin(),xsort2.end(),sortX);
        //find the average distance between the vecotr of points v1 and their closest neighbor in pc2
        return determine_offset_helper(v1,xsort2);
    }

    //transform the points in a pointcloud by translating it by x and y and rotating it by theta
    void transformPointCloud(sensor_msgs::msg::PointCloud &pc, double x, double y, double theta) {
        for(int i = 0; i < (int)pc.points.size(); i++) {
            //translate
            pc.points[i].set__x(pc.points[i].x+x);
            pc.points[i].set__y(pc.points[i].y+y);
            //rotate
            pc.points[i].set__x(cos(theta)*pc.points[i].x-sin(theta)*pc.points[i].y);
            pc.points[i].set__y(sin(theta)*pc.points[i].x+cos(theta)*pc.points[i].y);
        }
    }

    //same as transfrom pointcloud but makes a copy of the pointcloud instead of modifying it
    sensor_msgs::msg::PointCloud transformPointCloudCopy(sensor_msgs::msg::PointCloud pc, double x, double y, double theta) {
        sensor_msgs::msg::PointCloud pcCopy = sensor_msgs::msg::PointCloud();
        for(int i = 0; i < (int)pc.points.size(); i++) {
            geometry_msgs::msg::Point32 p = geometry_msgs::msg::Point32();
            p.set__x(pc.points[i].x);
            p.set__y(pc.points[i].y);
            p.set__z(pc.points[i].z);
            pcCopy.points.push_back(p);
        }
        transformPointCloud(pcCopy,x,y,theta);
        return pcCopy;
    }
    //class varible declaration
    sensor_msgs::msg::PointCloud lastPoints;
    sensor_msgs::msg::PointCloud allPoints;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> pointCloudSub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud>> pointCloudPublisher_;
    double x_pos, y_pos, theta;
};

//main method
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //create and run the node
  rclcpp::spin(std::make_shared<OdometryFromLidar>());
  rclcpp::shutdown();
  return 0;
}
