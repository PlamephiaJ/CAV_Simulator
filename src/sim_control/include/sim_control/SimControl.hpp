

#ifndef SIM_CONTROL_HPP
#define SIM_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <termios.h>
#include <unistd.h>


class SimControl : public rclcpp::Node
{
private:
    std::string contorl_topic_ = "/sim_control";
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_control_;

    char start_key_;
    char stop_key_;

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    struct termios orig_termios_;

    void on_timer();
    void start_sim();
    void stop_sim();
    
public:
    SimControl();
    ~SimControl();
};

#endif