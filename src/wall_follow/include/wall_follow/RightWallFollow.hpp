/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-07-01 18:23:53
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 18:35:45
 * @FilePath: /wall_follow/include/wall_follow/RightWallFollow.hpp
 * @Description: Right wall follow header file.
 */

#ifndef RIGHT_WALL_FOLLOW_HPP
#define RIGHT_WALL_FOLLOW_HPP

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/string.hpp"

class RightWallFollow : public rclcpp::Node {
public:
    RightWallFollow();

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

    /**
     * given a direction, return a range data.
     * @param range_data: vector received from LiDar.
     * @param direction: requested direction.
     * @return: a range.
    */
    double get_distance(const std::vector<float>& range_data, const double direction);

    /**
     * speed control based on steering_angle. Large angle, low speed; small angle, high speed.
     * @param steering_direction: direction that will be published to steering wheel
     * @return: speed [m/s].
    */
    double get_speed(double steering_direction);

    /**
     * calculate the error between the desired_distance_to_wall and the current position.
     * @param range_data: vector received from LiDar.
     * @return: an error.
    */
    double get_error(const std::vector<float>& range_data);

    /**
     * calculate the steering angle and publish the control signal.
     * @param error: calculated error.
    */
    void pid_control(double error);

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    double degrees_to_radians(double degrees);
};

#endif