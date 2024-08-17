/*
Created by Yuhao Chen on Jul 30, 2024.
Debug node.
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/string.hpp"


using std::placeholders::_1;

class ListenerNode : public rclcpp::Node {
public:
    ListenerNode() : Node("listener_node")
    {
        subscription_talker_ = this->create_subscription<std_msgs::msg::String>("/my_talker_topic", 10, std::bind(&ListenerNode::listen_callback, this, _1));
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_talker_;

    void listen_callback(const std_msgs::msg::String::ConstSharedPtr msg) 
    {   
        RCLCPP_INFO_STREAM(this->get_logger(), "Receive time: " << get_current_timestring() << std::endl << "Received: " << msg->data);
    }

    std::string get_current_timestring()
    {
        auto current_time = this->now();
        int64_t nanoseconds = current_time.nanoseconds();
        int64_t seconds = nanoseconds / 1000000000;
        int64_t minutes = seconds / 60;
        int64_t hours = minutes / 60;

        int hour = static_cast<int>(hours % 24);
        int minute = static_cast<int>(minutes % 60);
        int second = static_cast<int>(seconds % 60);
        int nano = static_cast<int>(nanoseconds % 1000000000);

        return std::to_string(hour) + ":" + std::to_string(minute) + ":" + std::to_string(second) + ":" + std::to_string(nano);
    }
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ListenerNode>());
    rclcpp::shutdown();
    return 0;
}