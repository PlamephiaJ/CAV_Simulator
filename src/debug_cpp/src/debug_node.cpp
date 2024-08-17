/*
Created by Yuhao Chen on May 25, 2024.
Debug node.
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/string.hpp"


using std::placeholders::_1;

class DebugNode : public rclcpp::Node {
public:
    DebugNode() : Node("debug_node")
    {
        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 1, std::bind(&DebugNode::odom_callback, this, _1));
        // subscription_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 1, std::bind(&DebugNode::scan_callback, this, _1));
    }

private:
    std::string odom_topic = "/car0/odom";
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;

    // std::string scan_topic = "/scan";
    // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_scan_;

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) 
    {   
        tf2::Quaternion quaternion( odom_msg->pose.pose.orientation.x, 
                                    odom_msg->pose.pose.orientation.y, 
                                    odom_msg->pose.pose.orientation.z, 
                                    odom_msg->pose.pose.orientation.w);

        double yaw, pitch, roll;
        tf2::Matrix3x3(quaternion).getEulerYPR(yaw, pitch, roll);
        RCLCPP_INFO(this->get_logger(), "current pose: '%f', '%f', '%f'", yaw, pitch, roll);
        RCLCPP_INFO(this->get_logger(), "current position: %f,  %f", odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y);
    }

//     void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
//     {
//         auto ranges = scan_msg->ranges;
//         auto amax = scan_msg->angle_max;
//         auto amin = scan_msg->angle_min;
//         auto aincre = scan_msg->angle_increment;
//         for (auto i = 0; i < 1080; i++)
//         {
//             double theta = i * aincre + amin;
//             if (rad_to_deg(theta) >= -90.0)
//             {
//                 RCLCPP_INFO(this->get_logger(), "i:  %d", i);
//                 break;
//             }
//         }
//         // double theta = 180 * aincre + amin;
//         // RCLCPP_INFO(this->get_logger(), "angle:  %f", rad_to_deg(theta));
//     }

    double rad_to_deg(const double rads)
    {
        return rads * 180.0 / M_PI;
    }



};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DebugNode>());
    rclcpp::shutdown();
    return 0;
}