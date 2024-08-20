/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-06-28 16:24:07
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-08-20 16:50:12
 * @FilePath: /pure_pursuit/src/pure_pursuit_node_sim.cpp
 * @Description: Read https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/lab5.html for detailed information.
 * This node needs a waypoints list, make sure to set the parameters in your launch file.
 * The node is designed for 2 cars simulation. However, it is very easy to extend it to more cars. You can design your own control node based on this node.
 */

#include "../include/pure_pursuit/PurePursuit.hpp"


geometry_msgs::msg::Point PurePursuit::tf_point_map_to_car(const geometry_msgs::msg::Point & point_mapframe)
{
    geometry_msgs::msg::PointStamped point_stamped_mapframe, point_stamped_carframe;
    point_stamped_mapframe.point = point_mapframe;
    tf2::doTransform(point_stamped_mapframe, point_stamped_carframe, tf2_transform_map_to_car_);
    return point_stamped_carframe.point;
}

geometry_msgs::msg::Point PurePursuit::tf_point_car_to_map(const geometry_msgs::msg::Point & point_carframe)
{
    geometry_msgs::msg::PointStamped point_stamped_mapframe, point_stamped_carframe;
    point_stamped_carframe.point = point_carframe;
    tf2::doTransform(point_stamped_carframe, point_stamped_mapframe, tf2_transform_car_to_map_);
    return point_stamped_mapframe.point;
}

void PurePursuit::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    if (!is_sim_start_)
    {
        return;
    }

    // look up the current transformation from map frame to car frame.
    try
    {
        tf2_transform_map_to_car_ = tf_buffer_->lookupTransform(car_frame_, map_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &e)
    {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", map_frame_, car_frame_, e.what());
        return;
    }

    // look up the current transformation from car frame to map frame.
    try
    {
        tf2_transform_car_to_map_ = tf_buffer_->lookupTransform(map_frame_, car_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &e)
    {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", car_frame_, map_frame_, e.what());
        return;
    }

    current_speed_ = odom_msg->twist.twist.linear.x;

    speed_vector_[current_car_index_] = current_speed_;

    // RCLCPP_INFO_STREAM(this->get_logger(), "Current speed: " << current_speed_);

    current_position_.x = odom_msg->pose.pose.position.x;
    current_position_.y = odom_msg->pose.pose.position.y;

    // filter the goal point candidates from all given navigation points. Goal point is represented in the car frame since it is easy to calculate.
    geometry_msgs::msg::Point goal_point_carframe;
    bool is_goal_point_valid = false;
    double min_distance = std::numeric_limits<double>::max();
    double goal_speed = 0.0;
    if (enable_speed_tracking_)
    {
        for (const auto & pas : position_and_speed_list_)
        {
            auto point_carframe = tf_point_map_to_car(pas.position);
            // we only consider the navigation points in the first and second quadrants.
            if (point_carframe.x > 0.0)
            {
                double distance = euclidean_distance(point_carframe, point_origin_);
                // to improve robustness, we do not solely consider the navigation points at the front lookahead distance, but instead increase a threshold space, causing the navigation point filter to become a ring rather than an arc.
                if (distance > distance_lookahead_ - half_ring_width_ && distance < distance_lookahead_ + half_ring_width_)
                {
                    // pick up the closest navigation point.
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                        goal_point_carframe = point_carframe;
                        goal_speed = pas.speed;
                        is_goal_point_valid = true;
                    }
                }
            }
        }
    }
    else
    {
        for (const auto & nav_point_mapframe : nav_points_mapframe_)
        {
            auto point_carframe = tf_point_map_to_car(nav_point_mapframe);
            // we only consider the navigation points in the first and second quadrants.
            if (point_carframe.x > 0.0)
            {
                double distance = euclidean_distance(point_carframe, point_origin_);
                // to improve robustness, we do not solely consider the navigation points at the front lookahead distance, but instead increase a threshold space, causing the navigation point filter to become a ring rather than an arc.
                if (distance > distance_lookahead_ - half_ring_width_ && distance < distance_lookahead_ + half_ring_width_)
                {
                    // pick up the closest navigation point.
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                        goal_point_carframe = point_carframe;
                        is_goal_point_valid = true;
                    }
                }
            }
        }
    }

    double steering_angle = 0.0;
    // if no suitable navigation point is filtered out, proceed straight ahead.
    if (is_goal_point_valid)
    {
        // visualize the goal point.
        debug_draw_point_in_rviz2(tf_point_car_to_map(goal_point_carframe));
        // debug_draw_arrow_in_rviz2(tf_point_car_to_map(point_origin), tf_point_car_to_map(goal_point_carframe));
        // see lab manual to understand the equation. The method is based on the single kinematic model.
        double gamma = 2.0 * goal_point_carframe.y / std::pow(distance_lookahead_, 2);
        // PID: P control.
        steering_angle = kp_ * gamma;
        // limit the steering angle.
        if (steering_angle > deg_to_rad(90.0))
        {
            steering_angle = deg_to_rad(90.0);
        }
        if (steering_angle < deg_to_rad(-90.0))
        {
            steering_angle = deg_to_rad(-90.0);
        }
    }
    else
    {
        geometry_msgs::msg::Point p_forward_carframe;
        p_forward_carframe.x = 1.0;
        p_forward_carframe.y = p_forward_carframe.z = 0.0;
        debug_draw_point_in_rviz2(tf_point_car_to_map(p_forward_carframe));
        // debug_draw_arrow_in_rviz2(tf_point_car_to_map(point_origin), tf_point_car_to_map(p_forward_carframe));
    }

    // RCLCPP_INFO(this->get_logger(), "steering angle: %f", rad_to_deg(steering_angle));

    // publish the ackermann drive message to control the car.
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.speed = get_speed(steering_angle, goal_speed);
    drive_msg.drive.steering_angle = steering_angle;
    publisher_drive_->publish(drive_msg);
}

double PurePursuit::euclidean_distance(const geometry_msgs::msg::Point & point1, const geometry_msgs::msg::Point & point2)
{
    return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2) + std::pow(point1.z - point2.z, 2));
}

double PurePursuit::get_speed(const double steering_angle, const double goal_speed)
{
    // if AEB starts, stop the car by publishing speed 0.0
    if (start_AEB_)
    {
        return 0.0;
    }
    else if (enable_speed_tracking_)
    {
        return (goal_speed + current_speed_) / 2.0;
    }
    else
    {
        double abs_angle = std::abs(steering_angle);

        if (abs_angle <= deg_to_rad(10))
        {
            return 2.0;
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
}

double PurePursuit::rad_to_deg(const double rads)
{
    return rads * 180.0 / M_PI;
}

double PurePursuit::deg_to_rad(const double degs)
{
    return degs * M_PI / 180.0;
}

void PurePursuit::debug_draw_point_in_rviz2(const geometry_msgs::msg::Point & point_mapframe)
{
    marker1_.header.frame_id = "map";
    marker1_.type = visualization_msgs::msg::Marker::SPHERE;
    marker1_.action = visualization_msgs::msg::Marker::ADD;
    marker1_.scale.x = 0.3; marker1_.scale.y = 0.3; marker1_.scale.z = 0.3;
    marker1_.color.r = 0.0; marker1_.color.g = 1.0; marker1_.color.b = 0.0; marker1_.color.a = 1.0;
    marker1_.action = visualization_msgs::msg::Marker::MODIFY;
    marker1_.id = 0;
    marker1_.header.stamp = this->get_clock().get()->now();
    marker1_.pose.position.x = point_mapframe.x; marker1_.pose.position.y = point_mapframe.y; marker1_.pose.position.z = 0.0;
    marker1_.pose.orientation.x = 0; marker1_.pose.orientation.y = 0; marker1_.pose.orientation.z = 0; marker1_.pose.orientation.w = 0;
    marker_publisher_->publish(marker1_);
}

void PurePursuit::debug_draw_arrow_in_rviz2(const geometry_msgs::msg::Point & point_start_mapframe, const geometry_msgs::msg::Point & point_end_mapframe)
{
    marker2_.header.frame_id = "map";
    marker2_.type = visualization_msgs::msg::Marker::ARROW;
    marker2_.action = visualization_msgs::msg::Marker::ADD;
    marker2_.points = {point_start_mapframe, point_end_mapframe};
    marker2_.scale.x = 0.1; marker2_.scale.y = 0.2;
    marker2_.id = 1;
    marker2_.color.r = 0.0; marker2_.color.g = 1.0; marker2_.color.b = 0.0; marker2_.color.a = 1.0;
    marker2_.header.stamp = this->get_clock().get()->now();
    marker_publisher_->publish(marker2_);
}

void PurePursuit::control_callback(const std_msgs::msg::String::ConstSharedPtr control_msg)
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

std::string PurePursuit::get_time_string()
{
    auto current_time = this->now();
    int64_t nanoseconds = current_time.nanoseconds();
    int64_t seconds = nanoseconds / 1000000000;
    int64_t minutes = seconds / 60;
    int64_t hours = minutes / 60;

    int hour = static_cast<int>(hours % 24);
    int minute = static_cast<int>(minutes % 60);
    int second = static_cast<int>(seconds % 60);
    int nano = static_cast<int>(nanoseconds % 1000000000);

    return std::to_string(hour) + ":" + std::to_string(minute) + ":" + std::to_string(second) + ":" + std::to_string(nano);
}

void PurePursuit::AEB_callback(const std_msgs::msg::String::ConstSharedPtr aeb_msg)
{

    if (aeb_msg->data == "AEB")
    {
        start_AEB_ = true;
        RCLCPP_INFO_STREAM(this->get_logger(), "AEB request received. Time:" << get_time_string());
        RCLCPP_INFO_STREAM(this->get_logger(), "AEB request content:" << aeb_msg->data);
    }
    else
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Ping received:" << aeb_msg->data << ". Time:" << get_time_string());
    }
}

double PurePursuit::calculate_TTC(const int& car_index1, const int& car_index2, const double& distance)
{
    const double angle1 = calculate_angle(car_index1, car_index2);
    const double speed1 = speed_vector_[car_index1] * std::cos(angle1);

    const double angle2 = calculate_angle(car_index2, car_index1);
    const double speed2 = speed_vector_[car_index2] * std::cos(angle2);
    
    const double iTTC = distance / (speed1 + speed2);

    return iTTC;
}

double PurePursuit::calculate_angle(const int& source_car_index, const int& target_car_index)
{
    const double x = v2v_transform_stamped_matrix_[source_car_index][target_car_index].transform.translation.x;
    const double y = v2v_transform_stamped_matrix_[source_car_index][target_car_index].transform.translation.y;

    return std::atan2(y, x);
}

void PurePursuit::opp_car_pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr opp_odom_msg)
{
    if (!is_emergency_brake_publisher_ || !lookup_transform())
    {
        return;
    }

    int opp_index = current_car_index_ == 0 ? 1 : 0;
    speed_vector_[opp_index] = opp_odom_msg->twist.twist.linear.x;

    double distance = euclidean_distance(opp_odom_msg->pose.pose.position, current_position_);

    // RCLCPP_INFO_STREAM(this->get_logger(), "opp_position: " << opp_odom_msg->pose.pose.position.x << " " << opp_odom_msg->pose.pose.position.y);

    if (AEB_MODE_ == "DISTANCE")
    {
        if (distance < AEB_DISTANCE_THRESHOLD_)
        {
            auto msg = std_msgs::msg::String();
            msg.data = "AEB";
            publisher_emergency_brake_->publish(msg);
            RCLCPP_INFO_STREAM(this->get_logger(), "AEB request sent. Time: " << get_time_string());
            return;
        }
    }
    else if (AEB_MODE_ == "TTC")
    {
        double TTC = calculate_TTC(current_car_index_, opp_index, distance);
        if (TTC > 0 && TTC < AEB_TTC_THRESHOLD_)
        {
            auto msg = std_msgs::msg::String();
            msg.data = "AEB";
            publisher_emergency_brake_->publish(msg);
            RCLCPP_INFO_STREAM(this->get_logger(), "AEB request sent. Time: " << get_time_string());
            return;
        }
    }

    auto current_time = this->now();
    if ((current_time - previous_timestamp_).seconds() > 2.0)
    {
        auto msg = std_msgs::msg::String();
        msg.data = "DELAY_CHECK_" + std::to_string(delay_count_);
        publisher_emergency_brake_->publish(msg);
        RCLCPP_INFO_STREAM(this->get_logger(), "Ping sent:" << msg.data << ". Time: " << get_time_string());
        previous_timestamp_ = current_time;
        delay_count_++;
    }
}

bool PurePursuit::lookup_transform()
{
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            if (i == j)
            {
                continue;
            }
            
            std::string i_frame = "car" + std::to_string(i) + "/base_link";
            std::string j_frame = "car" + std::to_string(j) + "/base_link";

            try
            {
               v2v_transform_stamped_matrix_[i][j] = tf_buffer_->lookupTransform(i_frame, j_frame, tf2::TimePointZero);
            }
            catch(const std::exception& e)
            {
                RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", i_frame, j_frame, e.what());
                return false;
            }
        }
    }
    return true;
}

PurePursuit::PurePursuit() : Node("pure_pursuit_node")
{
    this->declare_parameter("look_ahead_distance");
    this->declare_parameter("PID_P");
    this->declare_parameter("waypoint_file_path");
    this->declare_parameter("half_ring_width");
    this->declare_parameter("enable_speed_tracking");
    this->declare_parameter("is_emergency_brake_publisher");
    this->declare_parameter("AEB_MODE");
    this->declare_parameter("AEB_DISTANCE_THRESHOLD");
    this->declare_parameter("AEB_TTC_THRESHOLD");
    this->declare_parameter("ENABLE_DELAY_CHECK");

    drive_topic_ = this->get_namespace() + drive_topic_;
    odom_topic_ = this->get_namespace() + odom_topic_;
    car_frame_ = this->get_namespace() + car_frame_;
    car_frame_ = car_frame_.substr(1);

    current_car_index_ = this->get_namespace()[strlen(this->get_namespace()) - 1] - '0';
    RCLCPP_INFO_STREAM(this->get_logger(), "This car is car " << current_car_index_);

    publisher_drive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 1);
    subscription_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 1, std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1));
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("goalpoint_marker", 10);

    opp_odom_topic_ = strcmp(this->get_namespace(), "/car0") == 0 ? "/car1" + opp_odom_topic_ : "/car0" + opp_odom_topic_;
    subscription_opp_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(opp_odom_topic_, 1, std::bind(&PurePursuit::opp_car_pose_callback, this, std::placeholders::_1));

    point_origin_.x = point_origin_.y = point_origin_.z = current_position_.z = 0.0;

    distance_lookahead_ = this->get_parameter("look_ahead_distance").as_double();
    if (distance_lookahead_ < 0)
    {
        throw std::invalid_argument("Invalid parameter: look_ahead_distance must >= 0.");
    }

    kp_ = this->get_parameter("PID_P").as_double();
    if (kp_ < 0)
    {
        throw std::invalid_argument("Invalid parameter: PID_P must >= 0.");
    }

    half_ring_width_ = this->get_parameter("half_ring_width").as_double();
    if (half_ring_width_ < 0)
    {
        throw std::invalid_argument("Invalid parameter: half_ring_width must >= 0.");
    }

    enable_speed_tracking_ = this->get_parameter("enable_speed_tracking").as_bool();

    waypoint_file_path_ = this->get_parameter("waypoint_file_path").as_string();

    is_emergency_brake_publisher_ = this->get_parameter("is_emergency_brake_publisher").as_bool();

    AEB_MODE_ = this->get_parameter("AEB_MODE").as_string();
    if (!(AEB_MODE_ == "TTC" || AEB_MODE_ == "DISTANCE"))
    {
        throw std::invalid_argument("Invalid parameter: AEB_MODE must be 'TTC' or 'DISTANCE'.");
    }

    AEB_DISTANCE_THRESHOLD_ = this->get_parameter("AEB_DISTANCE_THRESHOLD").as_double();
    if (AEB_DISTANCE_THRESHOLD_ < 0)
    {
        throw std::invalid_argument("Invalid parameter: AEB_DISTANCE_THRESHOLD must >= 0.");
    }

    AEB_TTC_THRESHOLD_ = this->get_parameter("AEB_TTC_THRESHOLD").as_double();
    if (AEB_TTC_THRESHOLD_ < 0)
    {
        throw std::invalid_argument("Invalid parameter: AEB_TTC_THRESHOLD must >= 0.");
    }

    ENABLE_DELAY_CHECK_ = this->get_parameter("ENABLE_DELAY_CHECK").as_bool();

    // read the waypoints list.
    CSVHandler csv_handler(waypoint_file_path_);

    if (enable_speed_tracking_)
    {
        position_and_speed_list_ = csv_handler.read_waypoint_and_speed_list_from_csv();
    }
    else
    {
        nav_points_mapframe_ = csv_handler.read_waypoint_list_from_csv();
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscriber_control_ = this->create_subscription<std_msgs::msg::String>(control_topic_, 10, std::bind(&PurePursuit::control_callback, this, std::placeholders::_1));

    publisher_emergency_brake_ = this->create_publisher<std_msgs::msg::String>(emergency_brake_topic_, 10);
    subscription_emergency_brake_ = this->create_subscription<std_msgs::msg::String>(emergency_brake_topic_, 10, std::bind(&PurePursuit::AEB_callback, this, std::placeholders::_1));

    v2v_transform_stamped_matrix_.resize(2, std::vector<geometry_msgs::msg::TransformStamped>(2));
    speed_vector_.resize(2);

    previous_timestamp_ = this->now();
    delay_count_ = 0;

    RCLCPP_INFO_STREAM(this->get_logger(), "Node started successfully!");
}

PurePursuit::~PurePursuit() {}