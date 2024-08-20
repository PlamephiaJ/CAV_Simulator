/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-07-01 16:19:24
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 16:29:03
 * @FilePath: /auxiliary/include/auxiliary/WaypointRecorder.hpp
 * @Description: Waypoint recorder header file.
 */

#ifndef WAYPOINT_RECORDER_HPP
#define WAYPOINT_RECORDER_HPP

#include <fstream>
#include <termios.h>
#include <unistd.h>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class WaypointRecorder : public rclcpp::Node {
private:
    std::string file_path_;
    std::ofstream file_;
    std::string tmpfilename_ = "/sim_ws/waypoint/temp.csv";

    std::string odom_topic_ = "/odom";

    std::string mode_;
    char c_input_;

    bool enable_speed_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    struct termios orig_termios_;

    std::string map_frame_;
    std::string vehicle_frame_ = "/base_link";

    double sample_interval_s_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
    visualization_msgs::msg::Marker marker_;

    unsigned long long waypoint_num_ = 0;

    double current_speed_;

    void on_timer_auto();

    void on_timer_manual();

    std::string generate_waypoint_file_path();

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);

    
public:
    WaypointRecorder();
    ~WaypointRecorder();
};

#endif