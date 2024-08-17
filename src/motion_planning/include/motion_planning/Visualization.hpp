/*
 * @Author: Yuhao Chen
 * @Date: 2024-06-24 16:27:07
 * @LastEditors: Yuhao Chen
 * @LastEditTime: 2024-07-01 19:18:06
 * @Description: Visualization class.
 */

#pragma once

#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

static int num_visuals = 0;

class PointsVisualizer {
protected:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub;
    visualization_msgs::msg::Marker dots;
    std::string ns;
    std::string frame_id;

public:
    PointsVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, std::string ns, std::string frame_id, std_msgs::msg::ColorRGBA color, float scale);
    void add_point(geometry_msgs::msg::Point p);
    void publish_points();
};

class MarkerVisualizer {
protected:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub;
    visualization_msgs::msg::Marker dot;
    std::string ns;
    std::string frame_id;

public:
    MarkerVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, std::string ns, std::string frame_id, std_msgs::msg::ColorRGBA color, float scale, int shape);

    void set_pose(geometry_msgs::msg::Pose pose);
    void publish_marker();
};

#endif