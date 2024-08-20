/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-07-01 18:09:28
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 18:09:58
 * @FilePath: /pure_pursuit/src/main.cpp
 * @Description:
 */

#include "../include/pure_pursuit/PurePursuit.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}