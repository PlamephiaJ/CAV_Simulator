/*
 * @Author: Yuhao Chen
 * @Date: 2024-06-08 09:53:55
 * @LastEditors: Yuhao Chen
 * @LastEditTime: 2024-06-30 09:19:58
 * @Description: RRT* algorithm header file for F1Tenth. Make sure to understand RRT and RRT* first. You can find an introduction at https://f1tenth-coursekit.readthedocs.io/en/latest/lectures/ModuleD/lecture11.html
 */

#ifndef RRT_HPP
#define RRT_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include "../include/motion_planning/Visualization.hpp"

#include <random>

constexpr int RED = 1;
constexpr int GREEN = 2;
constexpr int BLUE = 3;

typedef struct RRT_Node {
    double x, y;
    double cost;
    int parent;
    std::vector<int> children;
    bool is_root = false;
} RRT_Node;

class RRT : public rclcpp::Node {
public:
    RRT();
    virtual ~RRT();
private:
    // parameters.
    // map inflation margin.
    float MARGIN_ = 0.2;
    // desired distance to goal.
    float DISTANCE_GOAL_AHEAD_ = 3.5;
    // laserscan dynamic obstacles detection range.
    float SCAN_RANGE_ = 4.0;
    // obstacles inflation margin.
    float DETECTED_OBS_MARGIN_ = 0.22;
    // minimum RRT iterations.
    int MAX_RRT_ITERATIONS_ = 1200;
    // maximum RRT iterations.
    int MIN_RRT_ITERATIONS_ = 1000;
    // RRT sample distribution standard deviation.
    float STD_ = 1.5;
    // RRT steering step size.
    float STEP_SIZE_ = 0.3;
    // RRT near nodes radius limitation.
    float NEAR_RANGE_ = 1.0;
    // goal radius limitation.
    float GOAL_TOLERANCE_ = 0.1;
    // RRT waypoints interval.
    float RRT_WAYPOINT_INTERVAL_ = 0.2;
    // pure pursuit look-ahead distance
    float DISTANCE_LOOK_AHEAD_ = 0.4;
    // pure pursuit PID P parameter.
    float PID_P_ = 0.25;

    // topic name
    std::string odom_topic_ = "/odom";
    std::string map_topic_ = "/map";
    std::string scan_topic_ = "/scan";
    std::string dynamic_map_topic_ = "/dynamic_map";
    std::string drive_topic_ = "/drive";

    std::string waypoint_file_path_;

    // random number generation.
    std::mt19937 gen_;
    std::uniform_real_distribution<> uni_dist_;

    // subscribers and callbacks.
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscriber_map_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;

    bool is_sim_start_ = false;
    std::string control_topic_ = "/sim_control";
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_control_;

    /**
     * @description: control the node.
     * @return {*}
     */
    void control_callback(const std_msgs::msg::String::ConstSharedPtr control_msg);
    
    /**
     * @description: map callback to receive initial map.
     * @param {nav_msgs::msg::OccupancyGrid::ConstSharedPtr} map_msg
     * @return {*}
     */
    void map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg);

    /**
     * @description: laserscan callback to detect dynamic obstacles.
     * @param {sensor_msgs::msg::LaserScan::ConstSharedPtr} scan_msg
     * @return {*}
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    /**
     * @description: odometry callback for RRT* main algorithm.
     * @param {nav_msgs::msg::Odometry::ConstSharedPtr} odom_msg
     * @return {*}
     */
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);

    // publishers.
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_dynamic_map_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_path_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_tree_node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_tree_branches_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_goal_visualizer_;

    // map and dynamic map.
    // timestamp recorded to clear dynamic obstacles.
    rclcpp::Time previous_map_updating_time_;

    // basic input map.
    nav_msgs::msg::OccupancyGrid map_;
    // dynamic updating map.
    nav_msgs::msg::OccupancyGrid dynamic_map_;
    // dynamic obstacle indices on dynamic map.
    std::vector<int> obstacle_indices_;

    /**
     * @description: clear dynamic obstacles.
     * @return {*}
     */
    void clear_map();

    // waypoints.
    std::vector<geometry_msgs::msg::Point> global_waypoints_;

    // RRT motion planning.
    // current goal index in the waypoints vector.
    int current_goal_index_;
    // current pose of the car.
    geometry_msgs::msg::Pose current_pose_;

    /**
     * @description: find current goal based on map and vehicle state. This is to optimize the waypoint searching efficiency.
     * @return {*}
     */
    void find_current_goal();

    /**
     * @description: find a proper initial goal.
     * @return {*}
     */
    void initialize_goal_waypoint();

    /**
     * @description: find a proper forward waypoint.
     * @return {*}
     */
    void find_ahead_goal_waypoint();

    /**
     * @description: given current pose and waypoint vector, find the closet waypoint.
     * @param {std::vector<geometry_msgs::msg::Point>&} waypoints
     * @param {geometry_msgs::msg::Pose&} pose
     * @return {int} closet waypoint index in waypoint vector.
     */
    int find_closest_ahead_waypoint(const std::vector<geometry_msgs::msg::Point>& waypoints, const geometry_msgs::msg::Pose& pose);

    /**
     * @description: sample a new point based on some constraints.
     * @return {std::pair<float, float>} new sample point coordinate (x, y) in map frame.
     */
    std::pair<float, float> sample();

    /**
     * @description: find nearest node to the sample point in the RRT tree.
     * @param {std::vector<RRT_Node>&} tree : RRT tree
     * @param {std::pair<float, float>&} sampled_point
     * @return {int} nearest node index in the RRT tree vector.
     */
    int nearest(std::vector<RRT_Node>& tree, std::pair<float, float>& sampled_point);

    /**
     * @description: steer function.
     * @param {RRT_Node&} nearest_node
     * @param {std::pair<float, float>&} sampled_point
     * @return {RRT_Node} a new node that can be inserted in to RRT tree.
     */
    RRT_Node steer(RRT_Node& nearest_node, std::pair<float, float>& sampled_point);

    /**
     * @description: check if there is an obstacle between two nodes.
     * @param {RRT_Node&} nearest_node
     * @param {RRT_Node&} new_node
     * @return {bool}
     */
    bool check_collision(RRT_Node& nearest_node, RRT_Node& new_node);

    /**
     * @description: find node's near neighbours.
     * @param {std::vector<RRT_Node>&} tree
     * @param {RRT_Node&} node
     * @return {std::vector<int>} near neighbour indices in RRT tree vector.
     */
    std::vector<int> near(std::vector<RRT_Node>& tree, RRT_Node& node);

    /**
     * @description: euclidean distance between two nodes.
     * @param {RRT_Node&} n1
     * @param {RRT_Node&} n2
     * @return {double}
     */
    double line_cost(RRT_Node& n1, RRT_Node& n2);

    /**
     * @description: check if the new node is closed to goal enough.
     * @param {RRT_Node&} new_node
     * @param {float} x_goal
     * @param {float} y_goal
     * @return {bool}
     */
    bool is_goal(RRT_Node& new_node, float x_goal, float y_goal);

    /**
     * @description: find the path in the RRT tree.
     * @param {std::vector<RRT_Node>&} tree
     * @param {RRT_Node&} node
     * @return {std::vector<RRT_Node>} a vector contains all nodes on the path.
     */
    std::vector<RRT_Node> find_path(std::vector<RRT_Node>& tree, RRT_Node& node);
    
    /**
     * @description: pure pursuit to follow the path.
     * @param {nav_msgs::msg::Path&} path
     * @return {*}
     */
    void follow_the_path(const nav_msgs::msg::Path& path);

    /**
     * @description: get speed based on steering angle.
     * @param {float} steering_angle
     * @return {float} speed
     */
    float get_speed(const float steering_angle);

    // tf2 transformation.
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string laser_frame_ = "/laser";
    std::string map_frame_ = "map";
    std::string car_frame_ = "/base_link";

    geometry_msgs::msg::TransformStamped tf2_transform_laser_to_map_;
    geometry_msgs::msg::TransformStamped tf2_transform_map_to_laser_;
    geometry_msgs::msg::TransformStamped tf2_transform_map_to_car_;
    geometry_msgs::msg::TransformStamped tf2_transform_car_to_map_;

    geometry_msgs::msg::Point tf_point_laser_to_map(const geometry_msgs::msg::Point& point_laserframe);
    geometry_msgs::msg::Point tf_point_map_to_laser(const geometry_msgs::msg::Point& point_mapframe);
    geometry_msgs::msg::Point tf_point_map_to_car(const geometry_msgs::msg::Point& point_mapframe);
    geometry_msgs::msg::Point tf_point_car_to_map(const geometry_msgs::msg::Point& point_carframe);

    /**
     * @description: update all TransformStamped.
     * @return {*}
     */
    bool lookup_transform();

    // visualization.
    MarkerVisualizer* goal_visualizer_;
    visualization_msgs::msg::Marker tree_nodes_;
    visualization_msgs::msg::Marker tree_branch_;

    void visualize_tree(std::vector<RRT_Node>& tree);

    // debug visualization
    std::string marker_frame_id_ = "map";
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_marker_;
    visualization_msgs::msg::Marker marker_point_;
    visualization_msgs::msg::Marker marker_line_;
    int32_t unique_point_id_ = 0;
    int32_t unique_line_id_ = 0;
    int32_t unique_tree_id_ = 0;

    void init_marker();
    // if time_to_live < 0, it will never disappear.
    void draw_point(const geometry_msgs::msg::Point & point, double time_to_live, int color);
    void draw_line(const std::vector<geometry_msgs::msg::Point> & points, double time_to_live, int color);
    void draw_point(const RRT_Node & point, double time_to_live, int color);
    void draw_line(const std::vector<RRT_Node> & points, double time_to_live, int color);
    void clear_marker();
};

#endif