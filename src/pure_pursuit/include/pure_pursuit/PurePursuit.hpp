/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-07-01 16:09:04
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-08-20 16:48:14
 * @FilePath: /pure_pursuit/include/pure_pursuit/PurePursuit.hpp
 * @Description: Pure pursuit header file.
 */

#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "../include/pure_pursuit/FileHandler.hpp"


class PurePursuit : public rclcpp::Node
{
private:
    std::string drive_topic_ = "/drive";
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive_;

    std::string odom_topic_ = "/odom";
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odometry_;

    std::string opp_odom_topic_ = "/odom";
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_opp_odometry_;

    // emergency brake topic, will be published and received through NS3 channel.
    std::string emergency_brake_topic_ = "/emergency_brake";
    // set true for the car which will publish AEB command.
    bool is_emergency_brake_publisher_ = false;
    // flag for starting AEB, actuation part will stop the car when it is set to true.
    bool start_AEB_ = false;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_emergency_brake_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_emergency_brake_;

    // node parameters.
    // look-ahead distance to calculate the curvature of the arc.
    double distance_lookahead_;
    // the half width of the filter ring. See the details in the corresponding function.
    double half_ring_width_;
    // PID control parameter P.
    double kp_;
    // enable speed tracking.
    bool enable_speed_tracking_;
    // AEB mode, TTC (time to collision) or distance
    std::string AEB_MODE_;

    double AEB_DISTANCE_THRESHOLD_;
    double AEB_TTC_THRESHOLD_;

    std::string waypoint_file_path_;
    std::ifstream file_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // frame id may be changed in different simulator environments.
    std::string car_frame_ = "/base_link";
    std::string map_frame_ = "map";
    geometry_msgs::msg::TransformStamped tf2_transform_map_to_car_;
    geometry_msgs::msg::TransformStamped tf2_transform_car_to_map_;

    // V2V transform stamps, use this to calculate TTC
    std::vector<std::vector<geometry_msgs::msg::TransformStamped>> v2v_transform_stamped_matrix_;

    // speed info corresponding to car0, car1, ...
    // using for TTC calculation
    std::vector<double> speed_vector_;
    // current car index corresponding to simulation environment
    int current_car_index_;

    // point_[xxxframe] means the coordinate of this point refers to the [xxxframe].
    std::vector<geometry_msgs::msg::Point> nav_points_mapframe_;
    std::vector<PosAndSpeed> position_and_speed_list_;

    // origin point [0, 0, 0]
    geometry_msgs::msg::Point point_origin_;

    double current_speed_;
    geometry_msgs::msg::Point current_position_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    visualization_msgs::msg::Marker marker1_;
    visualization_msgs::msg::Marker marker2_;

    // simulation unified control
    bool is_sim_start_ = false;
    std::string control_topic_ = "/sim_control";
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_control_;

    /**
     * 2 cars' TTC calculation.
     * @param car_index1
     * @param car_index2
     * @param distance: distance between 2 cars.
     * @return TTC between two cars.
    */
    double calculate_TTC(const int& car_index1, const int& car_index2, const double& distance);

    /**
     * @description: the callback will be called after receiving opponent car's pose and send AEB when necessary.
     * @return {*}
     */
    void opp_car_pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr opp_odom_msg);

    /**
     * @description: AEB callback for collision avoidance simulation scenario.
     * @return {*}
     */
    void AEB_callback(const std_msgs::msg::String::ConstSharedPtr aeb_msg);

    /**
     * @description: control the node.
     * @return {*}
     */
    void control_callback(const std_msgs::msg::String::ConstSharedPtr control_msg);

    /**
     * rigid body transformation. Given a point in the map frame, return a point in the car frame.
     * @param point_mapframe: geometry_msgs::msg::Point object. Coordinate refers to map frame.
     * @return geometry_msgs::msg::Point object. Coordinate refers to car frame.
    */
    geometry_msgs::msg::Point tf_point_map_to_car(const geometry_msgs::msg::Point & point_mapframe);

    /**
     * rigid body transformation. Given a point in the car frame, return a point in the map frame.
     * @param point_carframe: geometry_msgs::msg::Point object. Coordinate refers to car frame.
     * @return geometry_msgs::msg::Point object. Coordinate refers to map frame.
    */
    geometry_msgs::msg::Point tf_point_car_to_map(const geometry_msgs::msg::Point & point_carframe);

    /**
     * pose callback function. We do not use odom_msg in the simulator because we already known the TF2 between the car frame and map frame.
    */
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);

    /**
     * euclidean distance calculation.
     * @param point1: geometry_msgs::msg::Point object.
     * @param point2: geometry_msgs::msg::Point object.
     * @return euclidean distance between two points.
    */
    double euclidean_distance(const geometry_msgs::msg::Point & point1, const geometry_msgs::msg::Point & point2);

    /**
     * speed control based on steering_angle. Large angle, low speed; small angle, high speed. Modify speed setting based on your map.
     * @param steering_angle: direction that will be published to steering wheel
     * @return: speed [m/s].
    */
    double get_speed(const double steering_angle, const double goal_speed);

    double rad_to_deg(const double rads);

    double deg_to_rad(const double degs);

    /**
     * draw a point in the rviz2 simulator. This is to help goal point visualization.
     * @param point_mapframe: geometry_msgs::msg::Point object refers to map frame.
    */
    void debug_draw_point_in_rviz2(const geometry_msgs::msg::Point & point_mapframe);

    /**
     * draw an arrow in the rviz2 simulator. This is to help goal point visualization.
     * @param point_start_mapframe: geometry_msgs::msg::Point object refers to map frame.
     * @param point_end_mapframe: geometry_msgs::msg::Point object refers to map frame.
    */
    void debug_draw_arrow_in_rviz2(const geometry_msgs::msg::Point & point_start_mapframe, const geometry_msgs::msg::Point & point_end_mapframe);

    /**
     * use this to update v2v transform stamps.
     */
    bool lookup_transform();

    /**
     * calculate opp car's angle in the current car base frame.
     */
    double calculate_angle(const int& source_car_index, const int& target_car_index);

    /**
     * calculate time to collision, based on poses of 2 cars and distance between them.
     */
    double calculate_TTL(const int& car_index1, const int& car_index2, const double& distance);

    /**
     * transform ROS2 timestamp to hour, minute, second, nanosecond for logging purpose.
     */
    std::string get_time_string();

    // used to check if channel delay works properly. 
    rclcpp::Time previous_timestamp_;

    bool ENABLE_DELAY_CHECK_ = true;

    int delay_count_ = 0;

public:
    PurePursuit();
    ~PurePursuit();
};

#endif