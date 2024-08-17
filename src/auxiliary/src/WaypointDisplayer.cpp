/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-06-28 16:24:07
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 16:30:38
 * @FilePath: /auxiliary/src/waypoints_displayer_sim.cpp
 * @Description: Auxiliary node for displaying the waypoints in the simulator. This positions refer to map frame. Make sure to have /sim_ws/waypoint directory to store the position csv files.
 */

#include "../include/auxiliary/WaypointDisplayer.hpp"
#include "../include/auxiliary/FileHandler.hpp"

WaypointDisplayer::WaypointDisplayer() : Node("waypoint_displayer_node")
{
    this->declare_parameter("map_frame_name");
    this->declare_parameter("waypoint_file_path");
    this->declare_parameter("color_r");
    this->declare_parameter("color_g");
    this->declare_parameter("color_b");

    map_frame_ = this->get_parameter("map_frame_name").as_string();
    r_ = this->get_parameter("color_r").as_double();
    g_ = this->get_parameter("color_g").as_double();
    b_ = this->get_parameter("color_b").as_double();

    waypoint_file_path_ = this->get_parameter("waypoint_file_path").as_string();
    file_.open(waypoint_file_path_);

    CSVHandler csv_handler(waypoint_file_path_);
    nav_points_ = csv_handler.read_waypoint_list_from_csv();

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("global_waypoints_display", 10);
    marker_.header.frame_id = "map";
    marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 0.05; marker_.scale.y = 0.1; marker_.scale.z = 0.1;
    marker_.color.r = r_; marker_.color.g = g_; marker_.color.b = b_; marker_.color.a = 0.5;
    marker_.action = visualization_msgs::msg::Marker::MODIFY;
    marker_.id = 0;
    timer_ = this->create_wall_timer(std::chrono::seconds(5), [this]() {return this->on_timer();});
}

void WaypointDisplayer::on_timer()
{
    marker_.points = nav_points_;
    marker_.header.stamp = this->get_clock().get()->now();
    marker_publisher_->publish(marker_);
}