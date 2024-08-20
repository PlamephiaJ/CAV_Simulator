/*
 * @Author: Y. Chen moyunyongan@gmail.com
 * @Date: 2024-07-01 16:56:01
 * @LastEditors: Y. Chen moyunyongan@gmail.com
 * @LastEditTime: 2024-07-01 16:57:33
 * @FilePath: /gap_follow/src/main.cpp
 * @Description:
 */

#include "../include/reactive_node/ReactiveFollowGap.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}