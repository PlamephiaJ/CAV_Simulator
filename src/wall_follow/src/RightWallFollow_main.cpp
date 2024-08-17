/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-07-01 18:29:53
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 18:30:27
 * @FilePath: /wall_follow/src/RightWallFollow_main.cpp
 * @Description:
 */

#include "../include/wall_follow/RightWallFollow.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RightWallFollow>());
    rclcpp::shutdown();
    return 0;
}