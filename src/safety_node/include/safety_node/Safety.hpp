/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-07-01 16:35:46
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 18:44:22
 * @FilePath: /safety_node/include/safety_node/Safety.hpp
 * @Description: Safety node header file.
 */

#ifndef SAFETY_HPP
#define SAFETY_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <algorithm>

using std::placeholders::_1;

// safety node class.
class Safety : public rclcpp::Node {
public:
    Safety();

private:
    float speed = 0.0;
    float threshold_ = 1.0;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laser_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odometry_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

    size_t size_laserscan_ = 1080;

    std::string scan_topic_ = "/scan";
    std::string odom_topic_ = "/odom";
    std::string drive_topic_ = "/drive";

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
   
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) ;

    void emergency_break(float ttc);
};

#endif