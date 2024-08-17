/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-07-07 16:06:04
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-07 16:06:48
 * @FilePath: /measure/src/main.cpp
 * @Description:
 */

#include "../include/measure/Measure.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Measure>());
    rclcpp::shutdown();
    return 0;
}