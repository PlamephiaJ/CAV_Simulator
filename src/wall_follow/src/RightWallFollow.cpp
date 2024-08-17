/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-06-28 16:24:07
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 18:34:17
 * @FilePath: /wall_follow/src/RightWallFollow.cpp
 * @Description: Read https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/lab3.html for detailed information.
 * All direction representations in this file follow the /LaserScan frame. A detailed explanation about frames and rigid body transformation can be found in https://f1tenth-coursekit.readthedocs.io/en/latest/lectures/ModuleA/lecture03.html.
 */

#include "../include/wall_follow/RightWallFollow.hpp"

RightWallFollow::RightWallFollow() : Node("right_wall_follow_node")
{
    // declare parameters.
    // direction_a: the direction of the a-beam. Ref: https://raw.githubusercontent.com/f1tenth/f1tenth_lab3_template/62a7a3d687d00ba1dd25cf7025c13a623bafdb5b//img/wall_following_lab_figure_1.png
    this->declare_parameter("direction_a");
    this->declare_parameter("desired_distance_to_wall");
    // distance_look_ahead: look ahead distance L.
    this->declare_parameter("distance_look_ahead");
    this->declare_parameter("kp");
    this->declare_parameter("kd");
    this->declare_parameter("ki");

    direction_a_ = this->get_parameter("direction_a").as_double();
    if (direction_a_ <= -M_PI_2 || direction_a_ > degrees_to_radians(-20.0))
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid configuration: direction_a.");
        return;
    }

    desired_distance_to_wall_ = this->get_parameter("desired_distance_to_wall").as_double();
    if (desired_distance_to_wall_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid configuration: desired_distance_to_wall.");
        return;
    }

    distance_look_ahead_ = this->get_parameter("distance_look_ahead").as_double();
    if (distance_look_ahead_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid configuration: distance_look_ahead.");
        return;
    }

    // PID control parameters.
    kp_ = get_parameter("kp").as_double();
    kd_ = get_parameter("kd").as_double();
    ki_ = get_parameter("ki").as_double();

    lidarscan_topic_ = this->get_namespace() + lidarscan_topic_;
    drive_topic_ = this->get_namespace() + drive_topic_;

    subscription_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic_, 10, std::bind(&RightWallFollow::scan_callback, this, std::placeholders::_1));
    publisher_drive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);

    current_time_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

    subscriber_control_ = this->create_subscription<std_msgs::msg::String>(control_topic_, 10, std::bind(&RightWallFollow::control_callback, this, std::placeholders::_1));

    RCLCPP_INFO_STREAM(this->get_logger(), "Node started successfully!");
}

void RightWallFollow::control_callback(const std_msgs::msg::String::ConstSharedPtr control_msg)
{
    if (control_msg->data == "stop")
    {
        is_sim_start_ = false;
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        msg.drive.speed = 0.0;
        msg.drive.steering_angle = 0.0;
        for (int i = 0; i < 5; i++)
        {
            publisher_drive_->publish(msg);
        }
        rclcpp::shutdown();
    }
    else if (control_msg->data == "start")
    {
        is_sim_start_ = true;
    }
    else
    {
        throw std::runtime_error("Invalid control message!");
    }
    
}

double RightWallFollow::get_distance(const std::vector<float>& range_data, const double direction)
{
    if (direction < angle_min_ || direction > angle_max_) return 0.0;

    size_t index = std::round((direction + angle_max_) / angle_increment_);
    index = std::min(index, size_laserscan_ - 1);
    return range_data[index];
}

double RightWallFollow::get_speed(double steering_direction)
{
    if (steering_direction >= -degrees_to_radians(10.0) && steering_direction <= degrees_to_radians(10.0)) return 1.5;
    else if (steering_direction >= -degrees_to_radians(20.0) && steering_direction <= degrees_to_radians(20.0)) return 1.0;
    else return 0.5;
}

double RightWallFollow::get_error(const std::vector<float>& range_data)
{
    double dir_a = direction_a_;
    // b-beam always heads towards right side.
    double dir_b = -M_PI_2;

    double distance_a, distance_b;
    // filter the Inf situation.
    do
    {
        distance_a = get_distance(range_data, dir_a);
        dir_a += angle_increment_;
    } while (std::isinf(distance_a));

    do
    {
        distance_b = get_distance(range_data, dir_b);
        dir_b += angle_increment_;
    } while (std::isinf(distance_b));
    
    double theta = dir_a - dir_b;
    double alpha = std::atan((distance_a * std::cos(theta) - distance_b) / (distance_a * std::sin(theta)));
    double distance_predict = distance_b * std::cos(alpha) + distance_look_ahead_ * std::sin(alpha);
    
    return desired_distance_to_wall_ - distance_predict;
}

void RightWallFollow::pid_control(double error)
{
    std::chrono::milliseconds prev_time_ms = current_time_ms_;
    current_time_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

    double dt_s = (current_time_ms_.count() - prev_time_ms.count()) / 1000.0;

    integral_ = prev_error_ * dt_s;
    double derivative = (error - prev_error_) / dt_s;

    double steering_angle = kp_ * error + ki_ * integral_ + kd_ * derivative;

    prev_error_ = error;

    // reduce vibration
    if (std::abs(error) < 0.1) steering_angle = 0.0;

    double speed = get_speed(steering_angle);

    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.speed = speed;
    drive_msg.drive.steering_angle = steering_angle;

    publisher_drive_->publish(drive_msg);
}

void RightWallFollow::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
{
    if (!is_sim_start_)
    {
        return;
    }
    
    size_laserscan_ = scan_msg->ranges.size();
    angle_increment_ = scan_msg->angle_increment;
    angle_min_ = scan_msg->angle_min;
    angle_max_ = scan_msg->angle_max;

    double error = get_error(scan_msg->ranges);
    RCLCPP_INFO(this->get_logger(), "ERROR: '%f'", error);
    pid_control(error);
}

double RightWallFollow::degrees_to_radians(double degrees)
{
    return degrees * (M_PI / 180.0);
}