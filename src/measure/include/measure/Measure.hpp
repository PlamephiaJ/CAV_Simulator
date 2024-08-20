/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-07-06 16:09:24
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-09 11:02:12
 * @FilePath: /measure/include/measure/Measure.hpp
 * @Description: Measure header file.
 */

#ifndef MEASURE_HPP
#define MEASURE_HPP

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <fstream>

class Measure : public rclcpp::Node {
private:
    // patameters
    int NUM_AGENTS_ = 2;
    int OBSERVATION_INTERVAL_MS_ = 40;

    // topics
    const std::string odom_topic_ = "/odom";
    const std::string collision_topic_ = "/collisions";

    // subscribers
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subscriber_odom_vector_;
    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr subscriber_collision_;

    // status vectors
    std::vector<geometry_msgs::msg::Pose> car_pose_vector_;
    std::vector<double> speed_vector_;
    std::vector<uint32_t> updating_time_vector_;
    std::vector<int8_t> collision_status_;
    
    std::vector<bool> received_all_car_positions_;
    bool are_all_car_positions_received_ = false;

    std::ofstream file_;

    // tf2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::vector<std::vector<geometry_msgs::msg::TransformStamped>> transform_stamped_matrix_;

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::chrono::milliseconds observation_interval_ms_ = std::chrono::milliseconds(OBSERVATION_INTERVAL_MS_);

    std::string generate_measure_file_path();
    
    std::string generate_measure_file_header();

    bool is_sim_start_ = false;
    std::string control_topic_ = "/sim_control";
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_control_;

    /**
     * @description: control the node.
     * @return {*}
     */
    void control_callback(const std_msgs::msg::String::ConstSharedPtr control_msg);

    /**
     * @description: update status vectors.
     * @param {int} car_index
     * @param {nav_msgs::msg::Odometry::SharedPtr} odom_msg
     * @return {*}
     */
    void process_odometry(const int car_index, const nav_msgs::msg::Odometry::SharedPtr odom_msg);

    /**
     * @description: update status vectors.
     * @param {std_msgs::msg::Int8MultiArray::SharedPtr} collision_msg
     * @return {*}
     */
    void collision_callback(const std_msgs::msg::Int8MultiArray::SharedPtr collision_msg);

    /**
     * @description: do an observation, record the status.
     * @return {*}
     */
    void on_timer();

    /**
     * @description: calculate euclidean distance.
     * @param {double&} x1
     * @param {double&} y1
     * @param {double&} x2
     * @param {double&} y2
     * @return {*}
     */
    double euclidean_distance(const double& x1, const double& y1, const double& x2, const double& y2);

    /**
     * @description: calculate TTC.
     * @param {int&} car_index1
     * @param {int&} car_index2
     * @param {double&} distance
     * @return {*}
     */
    double calculate_TTC(const int& car_index1, const int& car_index2, const double& distance);

    /**
     * @description: the angle of the target car coordinate in the source_car frame.
     * @param {int&} source_car_index
     * @param {int&} target_car_index
     * @return {*}
     */
    double calculate_angle(const int& source_car_index, const int& target_car_index);

    /**
     * @description: update tf2.
     * @return {*}
     */
    bool lookup_transform();

public:
    Measure();
    ~Measure();
};

#endif