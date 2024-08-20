/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-07-01 16:16:44
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 16:18:19
 * @FilePath: /auxiliary/src/WaypointDisplayer_main.cpp
 * @Description:
 */

#include "../include/auxiliary/WaypointDisplayer.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointDisplayer>());
    rclcpp::shutdown();
    return 0;
}