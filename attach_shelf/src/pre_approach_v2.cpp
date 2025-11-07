#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include <chrono>
#include <cstdio>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <ios>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach_v2_node") {
    // Declare and get parameters
    this->declare_parameter("obstacle", 0.3);
    this->declare_parameter("degrees", -90);
    this->declare_parameter("final_approach", false);

    obstacle_distance_ = this->get_parameter("obstacle").as_double();
    degrees_to_rotate_ = this->get_parameter("degrees").as_int();
    final_approach_ = this->get_parameter("final_approach").as_bool();

    // Initialize subscribers and publishers
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/diffbot_base_controller/odom", 10,
        std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);

    approach_shelf_client_ =
        this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");

    // Initialize state machine
    current_state_ = State::MOVING_FORWARD;
    front_distance_ = std::numeric_limits<double>::infinity();
    current_yaw_ = 0.0;
    target_yaw_ = 0.0;
    service_called_ = false;

    // Control loop timer - state machine
    control_timer_ = this->create_wall_timer(
        100ms, std::bind(&PreApproach::control_loop, this));

    RCLCPP_INFO(this->get_logger(),
                "Pre-approach started. CurState: %s Target: %.2f m, Rotation: "
                "%d degrees",
                state_to_string(current_state_).c_str(), obstacle_distance_,
                degrees_to_rotate_);
  }

private:
  // State machine states
  enum class State {
    MOVING_FORWARD,
    STOPPING,
    ROTATING,
    CALLING_SERVICE,
    WAITING_FOR_SERVICE,
    FINISHED
  };

  std::string state_to_string(State state) {
    switch (state) {
    case State::MOVING_FORWARD:
      return "MOVING_FORWARD";
    case State::STOPPING:
      return "STOPPING";
    case State::ROTATING:
      return "ROTATING";
    case State::CALLING_SERVICE:
      return "CALLING_SERVICE";
    case State::WAITING_FOR_SERVICE:
      return "WAITING_FOR_SERVICE";
    case State::FINISHED:
      return "FINISHED";
    default:
      return "UNKNOWN";
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Get front laser reading
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

  void control_loop() {
    auto twist_msg = geometry_msgs::msg::Twist();

    switch (current_state_) {
    case State::MOVING_FORWARD:
      handle_moving_forward(twist_msg);
      break;

    case State::STOPPING:
      handle_stopping(twist_msg);
      break;

    case State::ROTATING:
      handle_rotating(twist_msg);
      break;

    case State::CALLING_SERVICE:
      handle_calling_service(twist_msg);
      break;

    case State::WAITING_FOR_SERVICE:
      handle_waiting_for_service(twist_msg);
      break;

    case State::FINISHED:
      handle_finished(twist_msg);
      break;
    }

    cmd_vel_publisher_->publish(twist_msg);
  }

  void handle_moving_forward(geometry_msgs::msg::Twist &twist_msg) {
    double braking_distance = 0.30;
    double effective_stop_distance = obstacle_distance_ + braking_distance;

    if (front_distance_ <= obstacle_distance_) {
      // Obstacle detected - stop
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;

      current_state_ = State::STOPPING;

      RCLCPP_INFO(this->get_logger(),
                  "Obstacle detected at %.2f m. Stopping...", front_distance_);
    } else if (front_distance_ >= effective_stop_distance) {
      // Keep moving forward at higher speed
      twist_msg.linear.x = 0.6;
      twist_msg.angular.z = 0.0;
    } else {
      // Keep moving forward at lower speed
      twist_msg.linear.x = 0.1;
      twist_msg.angular.z = 0.0;
    }
  }

  void handle_stopping(geometry_msgs::msg::Twist &twist_msg) {
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;

    current_state_ = State::ROTATING;

    // Calculate target yaw
    double target_rotation_rad = degrees_to_rotate_ * M_PI / 180.0;
    target_yaw_ = current_yaw_ + target_rotation_rad;

    // Normalize angle to [-π, π]
    while (target_yaw_ > M_PI)
      target_yaw_ -= 2.0 * M_PI;
    while (target_yaw_ < -M_PI)
      target_yaw_ += 2.0 * M_PI;

    RCLCPP_INFO(this->get_logger(), "Starting rotation to %.1f degrees",
                target_yaw_ * 180.0 / M_PI);
  }

  void handle_rotating(geometry_msgs::msg::Twist &twist_msg) {
    double angle_diff = target_yaw_ - current_yaw_;

    // Normalize angle
    while (angle_diff > M_PI)
      angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI)
      angle_diff += 2.0 * M_PI;

    if (std::abs(angle_diff) < 0.017) // 1 degrees of tolarance
    {
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;

      if (final_approach_) {
        current_state_ = State::CALLING_SERVICE;
        RCLCPP_INFO(this->get_logger(),
                    "Rotation complete. Calling approach_shelf service...");
      } else {
        current_state_ = State::FINISHED;
        RCLCPP_INFO(this->get_logger(),
                    "Rotation complete. Final approach disabled.");
      }

      RCLCPP_INFO(this->get_logger(),
                  "Rotation completed. Final yaw: %.2f rad (%.1f deg)",
                  current_yaw_, current_yaw_ * 180.0 / M_PI);
    } else {
      twist_msg.linear.x = 0.0;
      if (std::abs(angle_diff) <= 0.2) {
        twist_msg.angular.z = (angle_diff > 0) ? 0.20 : -0.20; // ~11 deg/s
      } else {
        twist_msg.angular.z = (angle_diff > 0) ? 0.52 : -0.52; // ~30 deg/s
      }
    }
  }

  void handle_calling_service(geometry_msgs::msg::Twist &twist_msg) {
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    bool _print_flag = false;

    if (!service_called_) {
      // Wait for service to be available
      if (!approach_shelf_client_->wait_for_service(5s)) {
        if (_print_flag) {
          return;
        }
        RCLCPP_WARN(this->get_logger(),
                    "Waiting for /approach_shelf service...");
        _print_flag = true;
        return;
      }

      // Create request
      auto request =
          std::make_shared<attach_shelf::srv::GoToLoading::Request>();
      request->attach_to_shelf = final_approach_;

      // Send async request
      auto future = approach_shelf_client_->async_send_request(request);

      // Store future for later
      service_future_ = future.future.share();
      service_called_ = true;
      current_state_ = State::WAITING_FOR_SERVICE;

      RCLCPP_INFO(this->get_logger(),
                  "Service request sent with attach_to_shelf = %s",
                  final_approach_ ? "true" : "false");
    }
  }

  void handle_waiting_for_service(geometry_msgs::msg::Twist &twist_msg) {
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;

    // Check if service response is ready
    if (service_future_.wait_for(std::chrono::milliseconds(0)) ==
        std::future_status::ready) {
      auto response = service_future_.get();

      RCLCPP_INFO(this->get_logger(), "Service response received: complete=%s",
                  response->complete ? "true" : "false");

      current_state_ = State::FINISHED;
    }
  }

  void handle_finished(geometry_msgs::msg::Twist &twist_msg) {
    // Complete stop
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;

    // Prevent multiple shutdown
    static bool shutdown_initiated = false;

    if (!shutdown_initiated) {
      shutdown_initiated = true;
      RCLCPP_INFO(this->get_logger(),
                  "Pre-approach sequence completed. Shutting down node.");

      // Cancel main timer
      control_timer_->cancel();

      // Complete shutdown
      rclcpp::shutdown();
    }
  }

  double obstacle_distance_;
  int degrees_to_rotate_;
  bool final_approach_;
  State current_state_;
  double front_distance_;
  double current_yaw_;
  double target_yaw_;
  bool service_called_;

  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture service_future_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr
      approach_shelf_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}