#pragma once
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

namespace utils {

std::string pick_first_available_frame(
    const std::vector<std::string> &candidates,
    const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
    const rclcpp::Logger &logger);

std::string pick_first_available_topic(
    const std::vector<std::string> &candidates,
    const rclcpp::Node *node,
    const rclcpp::Logger &logger);

}  // namespace utils
