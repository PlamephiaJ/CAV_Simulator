/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-06-28 16:24:07
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 16:40:43
 */

#include "../include/safety_node/Safety.hpp"

Safety::Safety() : Node("safety_node")
{
    this->declare_parameter("ittc");

    scan_topic_ = this->get_namespace() + scan_topic_;
    odom_topic_ = this->get_namespace() + odom_topic_;
    drive_topic_ = this->get_namespace() + drive_topic_;

    subscription_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_, 10, std::bind(&Safety::scan_callback, this, _1));
    subscription_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 10, std::bind(&Safety::odom_callback, this, _1));
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 1);

    threshold_ = this->get_parameter("ittc").as_double();
    if (threshold_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid configuration: ittc must >= 0.");
        return;
    }
}

void Safety::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    speed = odom_msg->twist.twist.linear.x;
}


void Safety::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
{
    auto ranges = scan_msg->ranges;

    size_laserscan_ = ranges.size(); // 1080

    float range_rate;
    float iTTC_in_direction;
    float angle;
    float range;

    for (size_t i = 0; i < size_laserscan_; i++)
    {
        angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        range = ranges[i];
        range_rate = speed * std::cos(angle);

        if (range_rate > 0)
        {
            iTTC_in_direction = range / range_rate;
            if (iTTC_in_direction < threshold_) emergency_break(iTTC_in_direction);
        }
    }


}

void Safety::emergency_break(float ttc)
{
    auto message = ackermann_msgs::msg::AckermannDriveStamped();
    message.drive.speed = 0.0;
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Emergency Braking ttc: '%f'", ttc);
}