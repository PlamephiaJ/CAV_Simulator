/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-07-06 16:13:41
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-08-14 10:20:47
 * @FilePath: /measure/src/Measure.cpp
 * @Description: Measure source file.
 */

#include "../include/measure/Measure.hpp"

Measure::~Measure()
{
    file_.close();
}

Measure::Measure() : rclcpp::Node("measure_node")
{
    this->declare_parameter("NUM_AGENTS");
    NUM_AGENTS_ = this->get_parameter("NUM_AGENTS").as_int();
    if (NUM_AGENTS_ < 1)
    {
        throw std::invalid_argument("Invalid configuration: NUM_AGENTS(int) must >= 1");
    }

    this->declare_parameter("OBSERVATION_INTERVAL_MS");
    OBSERVATION_INTERVAL_MS_ = this->get_parameter("OBSERVATION_INTERVAL_MS").as_int();
    if (OBSERVATION_INTERVAL_MS_ < 0)
    {
        throw std::invalid_argument("Invalid configuration: OBSERVATION_INTERVAL_MS(int) must >= 0");
    }
    
    for (int i = 0; i < NUM_AGENTS_; i++)
    {
        const std::string odom_topic = "/car" + std::to_string(i) + odom_topic_;

        const auto subscription = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, 
        [this, i](const nav_msgs::msg::Odometry::SharedPtr odom_msg){
          process_odometry(i, odom_msg);
        });

        subscriber_odom_vector_.emplace_back(subscription);
    }

    subscriber_collision_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(collision_topic_, 10, std::bind(&Measure::collision_callback, this, std::placeholders::_1));

    car_pose_vector_.resize(NUM_AGENTS_);
    speed_vector_.resize(NUM_AGENTS_);
    updating_time_vector_.resize(NUM_AGENTS_);
    received_all_car_positions_.resize(NUM_AGENTS_);
    collision_status_.resize(NUM_AGENTS_);

    file_.open(generate_measure_file_path(), std::ios::out);

    if (!file_.is_open())
    {
        throw std::runtime_error("Unable to create CSV file.");
    }

    
    file_ << generate_measure_file_header() << "\n";

    timer_ = this->create_wall_timer(observation_interval_ms_, [this]() {return this->on_timer();});

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    transform_stamped_matrix_.resize(NUM_AGENTS_, std::vector<geometry_msgs::msg::TransformStamped>(NUM_AGENTS_));

    subscriber_control_ = this->create_subscription<std_msgs::msg::String>(control_topic_, 10, std::bind(&Measure::control_callback, this, std::placeholders::_1));
}

void Measure::control_callback(const std_msgs::msg::String::ConstSharedPtr control_msg)
{
    if (control_msg->data == "stop")
    {
        is_sim_start_ = false;
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

void Measure::process_odometry(const int car_index, const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    updating_time_vector_[car_index] = odom_msg->header.stamp.nanosec;
    speed_vector_[car_index] = odom_msg->twist.twist.linear.x;
    car_pose_vector_[car_index] = odom_msg->pose.pose;
    
    if (!are_all_car_positions_received_)
    {
        received_all_car_positions_[car_index] = true;
        for (const auto status : received_all_car_positions_)
        {
            if (!status) return;
        }
        are_all_car_positions_received_ = true;
    }
}

void Measure::collision_callback(const std_msgs::msg::Int8MultiArray::SharedPtr collision_msg)
{
    collision_status_ = collision_msg->data;
}

std::string Measure::generate_measure_file_path()
{
    time_t now = time(nullptr);
    char* curr_time = ctime(&now);

    std::string time_string = std::string(curr_time).substr(0, 24);
    for (auto &c : time_string)
    {
        if (c == ' ')
        {
            c = '_';
        }
    }

    std::string path = "/sim_ws/measure/measure_" + time_string + ".csv";
    RCLCPP_INFO(this->get_logger(), "%s", path);
    return path;
}

std::string Measure::generate_measure_file_header()
{
    std::string header = "timestamp,";

    for (int i = 0; i < NUM_AGENTS_; i++)
    {
        header += "car" + std::to_string(i) + "speed,";
    }

    for (int i = 0; i < NUM_AGENTS_; i++)
    {
        for (int j = i + 1; j < NUM_AGENTS_; j++)
        {
            header += "car" + std::to_string(i) + "car" + std::to_string(j) + "TTC,";
            header += "car" + std::to_string(i) + "car" + std::to_string(j) + "distance,";
            header += "car" + std::to_string(i) + "car" + std::to_string(j) + "collision,";
        }
    }

    header.pop_back();
    return header;
}

void Measure::on_timer()
{
    if (!is_sim_start_)
    {
        return;
    }

    if (!are_all_car_positions_received_ || !lookup_transform())
    {
        return;
    }

    double current_sec = this->get_clock()->now().seconds();
    std::string line = std::to_string(current_sec) + ",";

    for (int i = 0; i < NUM_AGENTS_; i++)
    {
        line += std::to_string(speed_vector_[i]) + ",";
    }

    for (int i = 0; i < NUM_AGENTS_; i++)
    {
        for (int j = i + 1; j < NUM_AGENTS_; j++)
        {
            const double distance = euclidean_distance(car_pose_vector_[i].position.x, car_pose_vector_[i].position.y, car_pose_vector_[j].position.x, car_pose_vector_[j].position.y);
            line += std::to_string(calculate_TTC(i, j, distance)) + ",";
            line += std::to_string(distance) + ",";
            if (collision_status_.at(i) == 1 && collision_status_.at(j) == 1)
            {
                line += "1,";
            }
            else
            {
                line += "0,";
            }
        }
    }

    line.pop_back();
    file_ << line << "\n";
}

double Measure::euclidean_distance(const double& x1, const double& y1, const double& x2, const double& y2)
{
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

// double rad_to_deg(const double rad)
// {
//     return rad * 180 / M_PI;
// }

double Measure::calculate_TTC(const int& car_index1, const int& car_index2, const double& distance)
{
    const double angle1 = calculate_angle(car_index1, car_index2);
    const double speed1 = speed_vector_[car_index1] * std::cos(angle1);

    const double angle2 = calculate_angle(car_index2, car_index1);
    const double speed2 = speed_vector_[car_index2] * std::cos(angle2);
    
    const double iTTC = distance / (speed1 + speed2);

    return iTTC;
}

double Measure::calculate_angle(const int& source_car_index, const int& target_car_index)
{
    const double x = transform_stamped_matrix_[source_car_index][target_car_index].transform.translation.x;
    const double y = transform_stamped_matrix_[source_car_index][target_car_index].transform.translation.y;

    return std::atan2(y, x);
}

bool Measure::lookup_transform()
{
    for (int i = 0; i < NUM_AGENTS_; i++)
    {
        for (int j = 0; j < NUM_AGENTS_; j++)
        {
            if (i == j)
            {
                continue;
            }
            
            std::string i_frame = "car" + std::to_string(i) + "/base_link";
            std::string j_frame = "car" + std::to_string(j) + "/base_link";

            try
            {
               transform_stamped_matrix_[i][j] = tf_buffer_->lookupTransform(i_frame, j_frame, tf2::TimePointZero);
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
