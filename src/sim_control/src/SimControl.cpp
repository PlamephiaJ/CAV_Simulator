#include "../include/sim_control/SimControl.hpp"

SimControl::SimControl() : Node("sim_control_node")
{
    this->declare_parameter("CONTROL_TOPIC");
    contorl_topic_ = this->get_parameter("CONTROL_TOPIC").as_string();

    this->declare_parameter("START_KEY");
    std::string c_input_string = this->get_parameter("START_KEY").as_string();
    start_key_ = c_input_string[0];

    this->declare_parameter("STOP_KEY");
    c_input_string = this->get_parameter("STOP_KEY").as_string();
    stop_key_ = c_input_string[0];

    publisher_control_ = this->create_publisher<std_msgs::msg::String>(contorl_topic_, 10);

    tcgetattr(STDIN_FILENO, &orig_termios_);
    struct termios new_termios = orig_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SimControl::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "Control node ready:\nPress '%c' to start the sim.\nPress '%c' to stop the sim.", start_key_, stop_key_);
}

SimControl::~SimControl()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    RCLCPP_INFO(this->get_logger(), "The sim has been stopped successfully.");
}

void SimControl::on_timer()
{
    char c;
    if (read(STDIN_FILENO, &c, 1) == 1)
    {
        if (c == stop_key_)
        {
            stop_sim();
            rclcpp::shutdown();
        }
        else if (c == start_key_)
        {
            start_sim();
            RCLCPP_INFO_STREAM(this->get_logger(), "Start command published!");
        }
    }
}

void SimControl::stop_sim()
{
    auto message = std_msgs::msg::String();
    message.data = "stop";
    publisher_control_->publish(message);
}

void SimControl::start_sim()
{
    auto message = std_msgs::msg::String();
    message.data = "start";
    publisher_control_->publish(message);
}