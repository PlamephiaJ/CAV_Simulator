#include "../include/sim_control/SimControl.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimControl>());
    rclcpp::shutdown();
    return 0;
}