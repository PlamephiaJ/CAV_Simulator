/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-07-01 18:22:39
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 18:33:55
 * @FilePath: /wall_follow/src/LeftWallFollow_main.cpp
 * @Description:
 */

#include "../include/wall_follow/LeftWallFollow.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LeftWallFollow>());
    rclcpp::shutdown();
    return 0;
}