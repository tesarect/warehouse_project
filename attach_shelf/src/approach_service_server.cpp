#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "memory"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ios>
#include <iterator>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <type_traits>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer() : Node("approach_service_server") {

    approach_shelf_service_ =
        this->create_service<attach_shelf::srv::GoToLoading>(
            "/approach_shelf",
            std::bind(&ApproachServiceServer::handle_approach_shelf, this, _1,
                      _2));

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ApproachServiceServer::laser_callback, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/diffbot_base_controller/odom", 10,
        std::bind(&ApproachServiceServer::odom_callback, this,
                  std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);

    // TF broadcaster
    cart_static_tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cartframe_broadcasted_ = false;
    current_yaw_ = 0.0;
    front_distance_ = std::numeric_limits<double>::infinity();

    RCLCPP_INFO(this->get_logger(),
                "Approach Shelf service Started @ `/approach_shelf`");
  }

private:
  // Ros2 Objs
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr
      approach_shelf_service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster>
      cart_static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  const float intensities_threshold_ = 7000.0f;
  bool cartframe_broadcasted_;
  double current_yaw_;
  double front_distance_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Store latest scan
    latest_scan_ = msg;

    // Update front distance
    int front_index = msg->ranges.size() / 2;
    front_distance_ = msg->ranges[front_index];
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Convert quaternion to yaw angle
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);
  }

  void handle_approach_shelf(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {

    RCLCPP_INFO(this->get_logger(),
                "Approach shelf service called. attach_to_shelf: %s",
                request->attach_to_shelf ? "true" : "false");

    // Check if we have laser scan data
    if (!latest_scan_) {
      RCLCPP_WARN(this->get_logger(), "No laser scan data available yet");
      response->complete = false;
      return;
    }

    // Static TF broadcast of cart_frame
    if (!cartframe_broadcasted_) {
      bool legs_found = detect_and_broadcast_cart_frame();

      if (!legs_found) {
        RCLCPP_WARN(this->get_logger(), "Could not detect 2 shelf legs");
        response->complete = false;
        return;
      }

      cartframe_broadcasted_ = true;
      RCLCPP_INFO(this->get_logger(), "Cart frame successfully broadcasted");
    }

    // If attach_to_shelf is true, perform final approach
    if (request->attach_to_shelf) {
      bool success = perform_final_approach();
      response->complete = success;
    } else {
      // Just detection and TF broadcast, no movement
      response->complete = true;
    }
  }

  bool detect_and_broadcast_cart_frame() {
    const std::vector<float> &intensity_data = latest_scan_->intensities;
    const std::vector<float> &ranges = latest_scan_->ranges;

    if (intensity_data.empty()) {
      RCLCPP_WARN(this->get_logger(), "No intensity data available");
      return false;
    }

    // Find shelf legs using intensities
    struct ShelfLeg {
      size_t start;
      size_t end;
    };

    std::vector<ShelfLeg> shelf_legs;
    auto iter = intensity_data.begin();

    // Find up to 2 shelf legs
    while (iter != intensity_data.end() && shelf_legs.size() < 2) {
      // Find start of a leg (high intensity)
      iter = std::find_if(iter, intensity_data.end(), [this](float val) {
        return val >= intensities_threshold_;
      });

      if (iter == intensity_data.end())
        break;

      size_t start = std::distance(intensity_data.begin(), iter);

      // Find end of the same leg (intensity drops)
      iter = std::find_if(iter, intensity_data.end(), [this](float val) {
        return val < intensities_threshold_;
      });

      size_t end = (iter == intensity_data.end())
                       ? intensity_data.size() - 1
                       : std::distance(intensity_data.begin(), iter) - 1;

      shelf_legs.push_back({start, end});

      RCLCPP_INFO(this->get_logger(), "Detected leg %zu: indices [%zu, %zu]",
                  shelf_legs.size(), start, end);
    }

    // Check if we found exactly 2 legs
    if (shelf_legs.size() != 2) {
      RCLCPP_WARN(this->get_logger(), "Found %zu shelf legs, need exactly 2",
                  shelf_legs.size());
      return false;
    }

    // Calculate midpoint between the two legs
    const ShelfLeg &left_leg = shelf_legs[0];
    const ShelfLeg &right_leg = shelf_legs[1];

    // Use the inner edges of the legs (left leg's end, right leg's start)
    size_t left_index = left_leg.end;
    size_t right_index = right_leg.start;

    if (left_index >= ranges.size() || right_index >= ranges.size()) {
      RCLCPP_WARN(this->get_logger(), "Index out of range");
      return false;
    }

    float range_left = ranges[left_index];
    float range_right = ranges[right_index];

    if (!std::isfinite(range_left) || !std::isfinite(range_right)) {
      RCLCPP_WARN(this->get_logger(), "Invalid range at leg edges");
      return false;
    }

    // Convert polar to cartesian coordinates
    float angle_left =
        latest_scan_->angle_min + left_index * latest_scan_->angle_increment;
    float angle_right =
        latest_scan_->angle_min + right_index * latest_scan_->angle_increment;

    float x1 = range_left * std::cos(angle_left);
    float y1 = range_left * std::sin(angle_left);

    float x2 = range_right * std::cos(angle_right);
    float y2 = range_right * std::sin(angle_right);

    float mid_x = (x1 + x2) / 2.0f;
    float mid_y = (y1 + y2) / 2.0f;

    // Create the cart midpoint in laser frame
    geometry_msgs::msg::PointStamped point_in_laser;
    point_in_laser.header.frame_id = latest_scan_->header.frame_id;
    point_in_laser.point.x = mid_x;
    point_in_laser.point.y = mid_y;
    point_in_laser.point.z = 0.0;

    // Transform to global frame ("odom")
    geometry_msgs::msg::PointStamped point_in_odom;
    try {
      point_in_odom = tf_buffer_->transform(point_in_laser, "odom",
                                            tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
      return false;
    }

    // Broadcast static TF from "odom" to "cart_frame"
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "cart_frame";
    tf_msg.transform.translation.x = point_in_odom.point.x;
    tf_msg.transform.translation.y = point_in_odom.point.y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;

    cart_static_tf_broadcaster_->sendTransform(tf_msg);

    RCLCPP_INFO(this->get_logger(),
                "Broadcasted cart_frame at (%.2f, %.2f) in odom frame (was "
                "(%.2f, %.2f) in laser frame)",
                point_in_odom.point.x, point_in_odom.point.y, mid_x, mid_y);

    return true;
  }

  bool perform_final_approach() {
    RCLCPP_INFO(this->get_logger(), "Starting final approach to shelf");

    std::string robot_frame = "robot_base_link";
    std::string target_frame = "cart_frame";

    geometry_msgs::msg::TransformStamped transform_stamped;

    try {
      // Wait up to 3 seconds for the transform
      if (!tf_buffer_->canTransform(robot_frame, target_frame,
                                    tf2::TimePointZero,
                                    std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(),
                     "Transform from %s to %s not available",
                     robot_frame.c_str(), target_frame.c_str());
        return false;
      }

      // Get the transform
      transform_stamped = tf_buffer_->lookupTransform(robot_frame, target_frame,
                                                      tf2::TimePointZero);

      RCLCPP_INFO(this->get_logger(), "`cart_frame` TF found: x=%.2f, y=%.2f",
                  transform_stamped.transform.translation.x,
                  transform_stamped.transform.translation.y);

    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "TF2 exception: %s", ex.what());
      return false;
    }

    // Navigate to cart_frame using the transform
    double target_x = transform_stamped.transform.translation.x;
    double target_y = transform_stamped.transform.translation.y;

    // Calculate distance and angle to target
    double distance_to_target =
        std::sqrt(target_x * target_x + target_y * target_y);
    double angle_to_target = std::atan2(target_y, target_x);

    RCLCPP_INFO(this->get_logger(),
                "Distance to cart_frame: %.2f m, Angle: %.2f rad (%.1f deg)",
                distance_to_target, angle_to_target,
                angle_to_target * 180.0 / M_PI);

    auto twist_msg = geometry_msgs::msg::Twist();

    // Rotate to face the target (if needed)
    if (std::abs(angle_to_target) > 0.05) { // > ~3 degrees
      RCLCPP_INFO(this->get_logger(), "Rotating to face cart_frame...");

      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = (angle_to_target > 0) ? 0.3 : -0.3;

      // Rotate for calculated time
      double rotation_time = std::abs(angle_to_target) / 0.3;
      auto start_time = this->now();

      while ((this->now() - start_time).seconds() < rotation_time) {
        cmd_vel_publisher_->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
      }
    }

    // Move forward to cart_frame position
    RCLCPP_INFO(this->get_logger(), "Moving forward to cart_frame (%.2f m)...",
                distance_to_target);

    twist_msg.linear.x = 0.3;
    twist_msg.angular.z = 0.0;

    // Move for calculated time to reach cart_frame
    double move_time = distance_to_target / 0.2;
    auto start_time = this->now();

    while ((this->now() - start_time).seconds() < move_time) {
      cmd_vel_publisher_->publish(twist_msg);
    }

    RCLCPP_INFO(this->get_logger(), "Reached cart_frame position");

    // Move forward additional 30cm to go under shelf
    RCLCPP_INFO(this->get_logger(), "Moving 30cm forward to go under shelf...");

    twist_msg.linear.x = 0.3;
    start_time = this->now();

    while ((this->now() - start_time).seconds() < 1.5) {
      // 0.2 m/s * 1.5s = 0.3m
      cmd_vel_publisher_->publish(twist_msg);
    }

    // Final stop
    twist_msg.linear.x = 0.0;
    cmd_vel_publisher_->publish(twist_msg);

    RCLCPP_INFO(this->get_logger(), "Robot positioned under shelf");

    // Lift the shelf
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    auto elevator_publisher =
        this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);
    rclcpp::sleep_for(
        std::chrono::milliseconds(200)); // Let publisher initialize

    auto elevator_msg = std_msgs::msg::String();
    elevator_msg.data = "";
    elevator_publisher->publish(elevator_msg);

    RCLCPP_INFO(this->get_logger(), "Shelf lifted successfully");

    return true;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachServiceServer>());
  rclcpp::shutdown();
  return 0;
}