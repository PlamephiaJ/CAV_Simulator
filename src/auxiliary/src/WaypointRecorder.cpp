/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-06-28 16:24:07
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 19:13:01
 * @FilePath: /auxiliary/src/WaypointRecorder.cpp
 * @Description: Auxiliary node for recording the vehicle position in the simulator.
 * This position is collected by using tf2 transformation.
 * Make sure to have /sim_ws/waypoint directory to store the position csv files.
 */

#include "../include/auxiliary/WaypointRecorder.hpp"

void WaypointRecorder::on_timer_auto()
{
    std::string target_frame = map_frame_;
    std::string source_frame = vehicle_frame_;

    geometry_msgs::msg::TransformStamped t;

    try
    {
        t = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    }
    catch(const tf2::TransformException & e)
    {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", target_frame, source_frame, e.what());
        return;
    }

    const auto x = t.transform.translation.x;
    const auto y = t.transform.translation.y;

    if (enable_speed_)
    {
        file_ << x << "," << y << "," << current_speed_ << "\n";
        RCLCPP_INFO(this->get_logger(), "Waypoint No.: %d, logging: %f, %f, %f.", waypoint_num_, x, y, current_speed_);
    }
    else
    {
        file_ << x << "," << y << "\n";
        RCLCPP_INFO(this->get_logger(), "Waypoint No.: %d, logging: %f, %f", waypoint_num_, x, y);
    }

    waypoint_num_++;
    marker_.id++;
    marker_.header.stamp = this->get_clock().get()->now();
    marker_.pose.position.x = x; marker_.pose.position.y = y; marker_.pose.position.z = 0;
    marker_.pose.orientation.x = 0; marker_.pose.orientation.y = 0; marker_.pose.orientation.z = 0; marker_.pose.orientation.w = 0;
    marker_publisher_->publish(marker_);
}

void WaypointRecorder::on_timer_manual()
{
    char c;
    if (read(STDIN_FILENO, &c, 1) == 1 && c == c_input_)
    {
        std::string target_frame = map_frame_;
        std::string source_frame = vehicle_frame_;

        geometry_msgs::msg::TransformStamped t;

        try
        {
            t = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        }
        catch(const tf2::TransformException & e)
        {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", target_frame.c_str(), source_frame.c_str(), e.what());
            return;
        }

        const auto x = t.transform.translation.x;
        const auto y = t.transform.translation.y;
        
        if (enable_speed_)
        {
            file_ << x << "," << y << "," << current_speed_ << "\n";
            RCLCPP_INFO(this->get_logger(), "Waypoint No.: %d, logging: %f, %f, %f.", waypoint_num_, x, y, current_speed_);
        }
        else
        {
            file_ << x << "," << y << "\n";
            RCLCPP_INFO(this->get_logger(), "Waypoint No.: %d, logging: %f, %f", waypoint_num_, x, y);
        }

        waypoint_num_++;
        marker_.id++;
        marker_.header.stamp = this->get_clock().get()->now();
        marker_.pose.position.x = x; marker_.pose.position.y = y; marker_.pose.position.z = 0;
        marker_.pose.orientation.x = 0; marker_.pose.orientation.y = 0; marker_.pose.orientation.z = 0; marker_.pose.orientation.w = 0;
        marker_publisher_->publish(marker_);
    }
}

std::string WaypointRecorder::generate_waypoint_file_path()
{
    time_t now = time(nullptr);
    char* curr_time = ctime(&now);

    std::string s = std::to_string(sample_interval_s_);
    s = s.substr(0, s.find('.') + 2);

    std::string time_string = std::string(curr_time).substr(0, 24);
    for (auto &c : time_string)
    {
        if (c == ' ')
        {
            c = '_';
        }
    }

    std::string path = "/sim_ws/waypoint/waypoints_" + s + "s_" + time_string + ".csv";
    RCLCPP_INFO_STREAM(this->get_logger(), path);
    return path;
}

void WaypointRecorder::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    current_speed_ = odom_msg->twist.twist.linear.x;
}

WaypointRecorder::WaypointRecorder() : Node("waypoint_recorder_node")
{
    this->declare_parameter("mode");
    this->declare_parameter("map_frame_name");
    this->declare_parameter("sample_interval_second");
    this->declare_parameter("manual_record_key");
    this->declare_parameter("enable_speed");

    map_frame_ = this->get_parameter("map_frame_name").as_string();
    if (mode_ == "auto")
    {
        vehicle_frame_ = this->get_namespace() + vehicle_frame_;
        vehicle_frame_ = vehicle_frame_.substr(1);
    }
    else
    {
        vehicle_frame_ = "car0" + vehicle_frame_;
    }

    mode_ = this->get_parameter("mode").as_string();
    if (!(mode_ == "auto" || mode_ == "manual"))
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid parameter: mode_ must be \"auto\" or \"manual\". Your configuration: %s", mode_);
        return;
    }

    sample_interval_s_ = this->get_parameter("sample_interval_second").as_double();
    if (sample_interval_s_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid parameter: sample_interval_second must >= 0.");
        return;
    }

    if (mode_ == "manual")
    {
        std::string c_input_string = this->get_parameter("manual_record_key").as_string();
        c_input_ = c_input_string[0];
    }

    enable_speed_ = this->get_parameter("enable_speed").as_bool();

    std::chrono::milliseconds interval_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(sample_interval_s_));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    file_path_ = generate_waypoint_file_path();

    std::ifstream file(tmpfilename_);
    if (file.good())
    {
        if (remove(tmpfilename_.c_str()) != 0)
        {
            throw "Couldn't remove existing temp file.";
        }
    }
    file.close();

    file_.open(tmpfilename_);

    if (!file_.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open the temp csv file_! Exiting...");
        return;
    }

    std::vector<std::string> column_names = {"x", "y"};
    if (enable_speed_)
    {
        column_names.emplace_back("speed");
    }

    for (int i = 0; i < (int)column_names.size(); i++)
    {
        file_ << column_names[i];
        if (i < (int)column_names.size() - 1)
        {
            file_ << ",";
        }
    }
    file_ << "\n";

    if (mode_ == "auto")
    {
        odom_topic_ = this->get_namespace() + odom_topic_;
    }
    else
    {
        odom_topic_ = "/car0" + odom_topic_;
    }

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_marker", 10);
    subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 1, std::bind(&WaypointRecorder::odom_callback, this, std::placeholders::_1));

    marker_.header.frame_id = "map";
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::SPHERE;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.scale.x = 0.2; marker_.scale.y = 0.2; marker_.scale.z = 0.2;
    marker_.color.r = 1.0; marker_.color.g = 0.0; marker_.color.b = 0.0; marker_.color.a = 0.5;
    marker_.action = visualization_msgs::msg::Marker::MODIFY;

    if (mode_ == "auto")
    {
        timer_ = this->create_wall_timer(interval_ms, [this]() {return this->on_timer_auto();});
    }
    else if (mode_ == "manual")
    {
        tcgetattr(STDIN_FILENO, &orig_termios_);
        struct termios new_termios = orig_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&WaypointRecorder::on_timer_manual, this));
    }
}

WaypointRecorder::~WaypointRecorder()
{
    if (mode_ == "manual")
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    }
    file_.close();

    RCLCPP_INFO(this->get_logger(), "Removing repeating element...");

    std::ifstream input_file(tmpfilename_);
    if (!input_file.is_open())
    {
        RCLCPP_INFO(this->get_logger(), "Couldn't open the temp file.");
        return;
    }

    std::ofstream output_file(file_path_);
    if (!output_file.is_open())
    {
        RCLCPP_INFO(this->get_logger(), "Couldn't open the output file.");
        input_file.close();
        return;
    }

    std::unordered_set<std::string> unique_lines;
    std::string line;
    while (std::getline(input_file, line))
    {
        if (unique_lines.insert(line).second)
        {
            output_file << line << "\n";
        }
    }
    output_file.close();

    if (remove(tmpfilename_.c_str()) != 0)
    {
        RCLCPP_INFO(this->get_logger(), "Couldn't remove the temp file. Please do it manually.");
    }

    input_file.close();
    RCLCPP_INFO(this->get_logger(), "file has been saved to: %s", file_path_.c_str());
}