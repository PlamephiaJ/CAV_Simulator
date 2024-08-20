/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-06-28 16:24:07
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 16:56:48
 * @FilePath: /gap_follow/src/ReactiveFollowGap.cpp
 * @Description: Read https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/lab4.html#doc-lab4 for detailed information.
 * All direction representations in this file follow the /LaserScan frame. A detailed explanation about frames and rigid body transformation can be found in https://f1tenth-coursekit.readthedocs.io/en/latest/lectures/ModuleA/lecture03.html.
 * This follow-the-gap algorithm implemented all 4 tweaks mentioned in the lecture slides. Reference: https://docs.google.com/presentation/u/0/?authuser=0&usp=slides_web.
 */

#include "../include/reactive_node/ReactiveFollowGap.hpp"

ReactiveFollowGap::ReactiveFollowGap() : Node("reactive_node")
{
    // declare parameters.
    // max_window: the limit of the virtual lidar ranges. See tweak 4 for more details.
    this->declare_parameter("max_window");
    this->declare_parameter("vehicle_width");
    // disparity_threshold: see tweak 2 for more details.
    this->declare_parameter("disparity_threshold");

    lidarscan_topic_ = this->get_namespace() + lidarscan_topic_;
    drive_topic_ = this->get_namespace() + drive_topic_;

    subscription_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic_, 1, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
    publisher_drive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 1);

    max_window_ = this->get_parameter("max_window").as_double();
    if (max_window_ <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid configuration: max_window must > 0.");
        return;
    }

    vehicle_half_width_ = this->get_parameter("vehicle_width").as_double() / 2.0;
    if (vehicle_half_width_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid configuration: vehicle_width must > 0.");
        return;
    }

    disparity_threshold_ = this->get_parameter("disparity_threshold").as_double();
    if (disparity_threshold_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid configuration: disparity_threshold must >= 0.");
        return;
    }

    subscriber_control_ = this->create_subscription<std_msgs::msg::String>(control_topic_, 10, std::bind(&ReactiveFollowGap::control_callback, this, std::placeholders::_1));

    RCLCPP_INFO_STREAM(this->get_logger(), "Node started successfully!");
}

void ReactiveFollowGap::control_callback(const std_msgs::msg::String::ConstSharedPtr control_msg)
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

void ReactiveFollowGap::preprocess_lidar(std::vector<float>& ranges)
{   
    auto range_raw = ranges;

    for (int i = 0; i < size_laserscan_; i++)
    {
        ranges[i] = (ranges[i] + prev_ranges_[i]) / 2.0;

        if (ranges[i] > max_window_)
        {
            ranges[i] = max_window_;
        }
    }

    prev_ranges_ = range_raw;
}

void ReactiveFollowGap::set_bubble(std::vector<float>& ranges)
{
    std::set<int> disparities = find_disparity_index(ranges);

    for (const auto& i_disparity : disparities)
    {
        // calculate the bubble radius dynamically. You can find my poorly drawing figure at https://drive.google.com/file/d/1H2etQA6_WXBQdwJQjXWVycM8vvwLlEax/view?usp=sharing to understand how it works.
        double theta = std::atan((vehicle_half_width_) / ranges[i_disparity]);
        // convert angle to laser beams.
        int bubble_radius = std::round(theta / angle_increment_);

        // 20 beams are extra safety reserve.
        int i_bubble_start = std::max(0, i_disparity - bubble_radius - 20);
        int i_bubble_end = std::min(size_laserscan_, i_disparity + bubble_radius + 20);

        for (auto i = i_bubble_start; i <= i_bubble_end; i++)
        {
            ranges[i] = 0.0;
        }
    }
}

std::set<int> ReactiveFollowGap::find_disparity_index(const std::vector<float>& ranges)
{
    std::set<int> indices;

    for (int i = 1; i < size_laserscan_; i++)
    {
        if (ranges[i - 1] > ranges[i] && ranges[i - 1] - ranges[i] >= disparity_threshold_)
        {
            indices.insert(i);
        }
        else if (ranges[i - 1] < ranges[i] && ranges[i] - ranges[i - 1] >= disparity_threshold_)
        {
            indices.insert(i - 1);
        }
    }

    return indices;
}

std::pair<int, int> ReactiveFollowGap::find_max_gap(const std::vector<float>& ranges)
{   
    int i_start = -1;
    int i_end = -1;
    int current_start = 0;
    int current_length = 0;

    for (auto i = 0; i < size_laserscan_; i++)
    {
        // i.e. ranges[i] != 0.0
        if (ranges[i] > EPSILON)
        {
            if (i_start == -1)
            {
                i_start = i;
                i_end = i;
            }
            else
            {
                i_end = i;
            }
        }
        else
        {
            if (i_start != -1)
            {
                if (current_length < i_end - i_start + 1)
                {
                    current_start = i_start;
                    current_length = i_end - i_start + 1;
                }
                i_start = -1;
                i_end = -1;
            }
        }
    }

    if (i_start != -1 && current_length < i_end - i_start + 1)
    {
        current_start = i_start;
        current_length = i_end - i_start + 1;
    }

    return std::make_pair(current_start, current_start + current_length - 1);
}

int ReactiveFollowGap::find_best_point(const std::vector<float>& ranges, const std::pair<int, int>& indices)
{   
    std::vector<int> idxs;
    
    auto it_max = std::max_element(ranges.begin() + indices.first, ranges.begin() + indices.second + 1);

    for (auto i = indices.first; i <= indices.second; i++)
    {
        // i.e. ranges[i] == max_range
        if (std::abs(ranges[i] - *it_max) < EPSILON)
        {
            idxs.emplace_back(i);
        }
    }

    return idxs[(int)std::round(idxs.size() / 2)];
}

double ReactiveFollowGap::get_speed(const double steering_angle)
{
    double abs_angle = std::abs(steering_angle);

    if (abs_angle <= deg_to_rad(10))
    {
        return 3.0;
    }
    else if (abs_angle <= deg_to_rad(20))
    {
        return 1.0;
    }
    else
    {
        return 0.5;
    }
}

double ReactiveFollowGap::get_direction_rads(const std::vector<float>& ranges, const int index)
{
    double boundary = deg_to_rad(-90.0);
    for (auto i = 0; angle_min_ + i * angle_increment_ <= boundary; i++)
    {
        if (ranges[i] <= 0.1 * vehicle_half_width_) return 0.0;
    }
    boundary = deg_to_rad(90.0);
    for (auto i = 0; angle_max_ - i * angle_increment_ >= boundary; i++)
    {
        if (ranges[size_laserscan_ - 1 - i] <= 0.1 * vehicle_half_width_) return 0.0;
    }

    return angle_min_ + index * angle_increment_;
}

double ReactiveFollowGap::rad_to_deg(const double rads)
{
    return rads * 180.0 / M_PI;
}

double ReactiveFollowGap::deg_to_rad(const double degs)
{
    return degs * M_PI / 180.0;
}

void ReactiveFollowGap::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
{   
    if (!is_sim_start_)
    {
        return;
    }
    size_laserscan_ = scan_msg->ranges.size();
    angle_min_ = scan_msg->angle_min;
    angle_max_ = scan_msg->angle_max;
    angle_increment_ = scan_msg->angle_increment;
    auto ranges = scan_msg->ranges;

    preprocess_lidar(ranges);

    set_bubble(ranges);

    std::pair<int, int> indice_max_gap = find_max_gap(ranges);

    int index = find_best_point(ranges, indice_max_gap);

    double steering_angle = get_direction_rads(ranges, index);

    // RCLCPP_INFO(this->get_logger(), "steering_angle: '%f'", rad_to_deg(steering_angle));

    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.speed = get_speed(steering_angle);
    drive_msg.drive.steering_angle = steering_angle;

    publisher_drive_->publish(drive_msg);
}