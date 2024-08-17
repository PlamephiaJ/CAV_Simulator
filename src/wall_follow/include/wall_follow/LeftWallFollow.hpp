/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-07-01 18:15:07
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 18:35:21
 * @FilePath: /wall_follow/include/wall_follow/LeftWallFollow.hpp
 * @Description: Left wall follow header file.
 */

#ifndef LEFT_WALL_FOLLOW_HPP
#define LEFT_WALL_FOLLOW_HPP

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/string.hpp"

class LeftWallFollow : public rclcpp::Node {
public:
    LeftWallFollow();

private:
    std::chrono::milliseconds current_time_ms_;
    double prev_error_ = 0.0;
    double error_ = 0.0;
    double integral_ = 0.0;

    double kp_;
    double kd_;
    double ki_;

    float angle_increment_ = 0.0;
    float angle_min_ = 0.0;
    float angle_max_ = 0.0;
    size_t size_laserscan_;
    double direction_a_;
    double desired_distance_to_wall_;
    double distance_look_ahead_;

    std::string lidarscan_topic_ = "/scan";
    std::string drive_topic_ = "/drive";

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laser_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive_;

    bool is_sim_start_ = false;
    std::string control_topic_ = "/sim_control";
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_control_;

    /**
     * @description: control the node.
     * @return {*}
     */
    void control_callback(const std_msgs::msg::String::ConstSharedPtr control_msg);

    double degrees_to_radians(double degrees);

    double get_distance(const std::vector<float>& range_data, const double direction);

    double get_speed(double steering_direction);

    double get_error(const std::vector<float>& range_data);

    void pid_control(double error);

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
};

#endif