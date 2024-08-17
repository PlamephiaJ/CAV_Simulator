/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-07-01 16:45:52
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 18:42:59
 * @FilePath: /gap_follow/include/reactive_node/ReactiveFollowGap.hpp
 * @Description: Reactive node header file.
 */

#ifndef REACTIVE_FOLLOW_GAP_HPP
#define REACTIVE_FOLLOW_GAP_HPP

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/string.hpp"

// float number comparison tolerance
constexpr double EPSILON = 1e-6;

class ReactiveFollowGap : public rclcpp::Node {
public:
    ReactiveFollowGap();

private:
    std::string lidarscan_topic_ = "/scan";
    std::string drive_topic_ = "/drive";

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laser_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive_;

    int size_laserscan_ = 1080;
    double angle_increment_;
    double angle_min_;
    double angle_max_;
    double max_window_;
    double vehicle_half_width_;
    double disparity_threshold_;
    std::vector<float> prev_ranges_ = std::vector<float>(size_laserscan_, 0.0f);

    bool is_sim_start_ = false;
    std::string control_topic_ = "/sim_control";
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_control_;

    /**
     * @description: control the node.
     * @return {*}
     */
    void control_callback(const std_msgs::msg::String::ConstSharedPtr control_msg);

    /**
     * preprocess the raw lidar data, limit the visibility of the vehicle. This function will modify raw data instead of creating a copy to save memory.
     * @param ranges: vector received from LiDar.
    */
    void preprocess_lidar(std::vector<float>& ranges);

    /**
     * set range data near the disparity to zero. Make bubbles to avoid collision. This function will modify raw data instead of creating a copy to save memory.
     * @param ranges: virtual LiDAR data.
    */
    void set_bubble(std::vector<float>& ranges);

    /**
     * find every disparity in the range array. Always treat the smaller one as the center of the disparity.
     * @param ranges: virtual LiDAR data.
     * @return: a set of the disparity indices.
    */
    std::set<int> find_disparity_index(const std::vector<float>& ranges);

    /**
     * find the longest non-zero subarray's start and end indices.
     * @param ranges: virtual LiDAR data.
     * @return: a pair of the start index and end index.
    */
    std::pair<int, int> find_max_gap(const std::vector<float>& ranges);

    /**
     * find the middle point of the max ranges.
     * @param ranges: virtual LiDAR data.
     * @param indices: a pair of the start index and end index of the max gap.
     * @return: middle point index.
    */
    int find_best_point(const std::vector<float>& ranges, const std::pair<int, int>& indices);

    /**
     * speed control based on steering_angle. Large angle, low speed; small angle, high speed. This function can use range in that direction as well. Modify speed setting based on your map.
     * @param steering_angle: direction that will be published to steering wheel
     * @return: speed [m/s].
    */
    double get_speed(const double steering_angle);

    /**
     * given an index of the LiDAR array, return the real steering angle. Tweak 3 is used to avoid striking a corner.
     * @param ranges: virtual LiDAR data.
     * @param index: an index of the LiDAR array. Desired direction.
     * @return: angle [rads].
    */
    double get_direction_rads(const std::vector<float>& ranges, const int index);

    double rad_to_deg(const double rads);

    double deg_to_rad(const double degs);

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
};

#endif