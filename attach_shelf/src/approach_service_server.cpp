#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "memory"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
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
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <type_traits>
#include <vector>

// DYNAMIC INTENSITY THRESHOLD - currently working on this

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer() : Node("approach_service_server") {

    this->declare_parameter<std::string>("odom_frame");
    this->declare_parameter<std::string>("base_frame");
    this->declare_parameter<std::string>("cmd_vel_topic");
    this->declare_parameter<float>("align_ahead_dist");
    this->declare_parameter<float>("align_cart_center_dist");
    this->declare_parameter<double>("position_tolerance");
    this->declare_parameter<double>("angle_tolerance");
    this->declare_parameter<double>("angular_speed");
    this->declare_parameter<double>("linear_speed");
    this->declare_parameter<double>("shelf_center_distance");
    this->declare_parameter<double>("shelf_center_speed");

    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
    this->get_parameter("align_ahead_dist", align_ahead_dist_);
    this->get_parameter("align_cart_center_dist", align_cart_center_dist_);
    this->get_parameter("position_tolerance", position_tolerance_);
    this->get_parameter("angle_tolerance", angle_tolerance_);
    this->get_parameter("angular_speed", angular_speed_);
    this->get_parameter("linear_speed", linear_speed_);
    this->get_parameter("shelf_center_distance", shelf_center_distance_);
    this->get_parameter("shelf_center_speed", shelf_center_speed_);

    RCLCPP_INFO(this->get_logger(),
                "Parameters "
                "loaded:\n\todom_frame=%s\n\tbase_frame=%s\n\tcmd_vel_topic=%s",
                odom_frame_.c_str(), base_frame_.c_str(),
                cmd_vel_topic_.c_str());

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

    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    // TF broadcaster
    cart_ahead_static_tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    cart_entry_static_tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    cart_center_static_tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cartframe_broadcasted_ = false;
    alignframe_broadcasted_ = false;
    legs_found_ = false;
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
      cart_ahead_static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster>
      cart_entry_static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster>
      cart_center_static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  float intensities_threshold_ = 0.0f;
  float align_ahead_dist_;
  float align_cart_center_dist_;
  bool alignframe_broadcasted_;
  bool cartframe_broadcasted_;
  bool legs_found_;
  bool shelf_attached_;
  bool odom_received_ = false;
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;
  double front_distance_;
  float align_angle_threshold_ = 0.5;
  double align_angular_speed_;
  double position_tolerance_;
  double angular_speed_;
  double angle_tolerance_;
  double linear_speed_;
  double shelf_center_speed_;
  double shelf_center_distance_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string cmd_vel_topic_;
  const std::string cart_ahead_frame_ = "alignment_correction_frame";
  const std::string cart_entry_frame_ = "cart_entry_frame";
  const std::string cart_center_frame_ = "cart_frame";
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Store latest scan
    latest_scan_ = msg;

    // Update front distance
    int front_index = msg->ranges.size() / 2;
    front_distance_ = msg->ranges[front_index];
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // Extract yaw from quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);
    odom_received_ = true;
  }

  void handle_approach_shelf(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Approach shelf (attach_to_shelf): %s",
                request->attach_to_shelf ? "REQUESTED" : "NOT REQUESTED");

    if (!request->attach_to_shelf) {
      return;
    }

    // Check if we have laser scan data
    if (!latest_scan_) {
      RCLCPP_WARN(this->get_logger(), "No laser scan data available yet");
      response->complete = false;
      return;
    }

    // Static TF broadcast of cart_frame
    if (!cartframe_broadcasted_ && !alignframe_broadcasted_) {

      RCLCPP_INFO(this->get_logger(),
                  "Detecting Legs being requested as no legs found yet");
      legs_found_ = detect_and_broadcast_cart_frame();

      if (!legs_found_) {
        RCLCPP_WARN(this->get_logger(), "Could not detect 2 shelf legs");
        response->complete = false;
        return;
      }

      alignframe_broadcasted_ = true;
      RCLCPP_INFO(this->get_logger(), "Align frame successfully broadcasted");

      cartframe_broadcasted_ = true;
      RCLCPP_INFO(this->get_logger(), "Cart frame successfully broadcasted");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Align frame NOT broadcasted");
      RCLCPP_ERROR(this->get_logger(), "Cart frame NOT broadcasted");
    }

    // If attach_to_shelf is true, perform final approach
    if (request->attach_to_shelf) {
      bool success = perform_final_approach();
      response->complete = success;
    } else {
      // Just detection and TF broadcast, no movement
      response->complete = true; // ? doubtful Need to test this scenario
    }
  }

  bool detect_and_broadcast_cart_frame() {
    const std::vector<float> &intensity_data = latest_scan_->intensities;
    const std::vector<float> &ranges = latest_scan_->ranges;

    if (intensity_data.empty()) {
      RCLCPP_WARN(this->get_logger(), "No intensity data available");
      return false;
    }

    float max_intensity =
        *std::max_element(intensity_data.begin(), intensity_data.end());
    intensities_threshold_ = max_intensity * 0.75f;

    RCLCPP_INFO(this->get_logger(), "max intensity found to be : %f",
                max_intensity);
    RCLCPP_INFO(this->get_logger(), "Intensity Threshold to be : %f",
                intensities_threshold_);

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

    float center_entry_x = (x1 + x2) / 2.0f;
    float center_entry_y = (y1 + y2) / 2.0f;

    // Line that is perpendicular to legs passing through cart frame for
    // alignment Direction vector (leg to leg) and length
    float dx = x2 - x1;
    float dy = y2 - y1;
    float length = std::sqrt(dx * dx + dy * dy);

    // Perpendicular unit vector
    float perp_x = -dy / length;
    float perp_y = dx / length;

    // Alignment correction frame coordinates
    float ahead_offset = align_ahead_dist_;
    float ahead_x = center_entry_x + ahead_offset * perp_x;
    float ahead_y = center_entry_y + ahead_offset * perp_y;

    // Cart center frame coordinates
    float beyond_offset = align_cart_center_dist_;
    float cart_center_x = center_entry_x - beyond_offset * perp_x;
    float cart_center_y = center_entry_y - beyond_offset * perp_y;

    // Create the alignment frame before cart frame
    geometry_msgs::msg::PointStamped cart_ahead_point;
    cart_ahead_point.header.frame_id = latest_scan_->header.frame_id;
    cart_ahead_point.point.x = ahead_x;
    cart_ahead_point.point.y = ahead_y;
    cart_ahead_point.point.z = 0.0;

    // Transform to global frame (map)
    geometry_msgs::msg::PointStamped cart_ahead_tf_odom;
    try {
      cart_ahead_tf_odom = tf_buffer_->transform(cart_ahead_point, odom_frame_,
                                                 tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
      return false;
    }

    // Broadcast static TF from "odom" to "alignment_correction_frame"
    geometry_msgs::msg::TransformStamped cart_ahead_tf;
    cart_ahead_tf.header.stamp = this->get_clock()->now();
    cart_ahead_tf.header.frame_id = odom_frame_;
    cart_ahead_tf.child_frame_id = cart_ahead_frame_;
    cart_ahead_tf.transform.translation.x = cart_ahead_tf_odom.point.x;
    cart_ahead_tf.transform.translation.y = cart_ahead_tf_odom.point.y;
    cart_ahead_tf.transform.translation.z = 0.0;

    // Orientation: perpendicular direction (for proper heading)
    float align_yaw = std::atan2(ahead_y, ahead_x);

    // Rotate 90° left
    // align_yaw = align_yaw + M_PI_2; // pi/2
    align_yaw = align_yaw - M_PI_2; // pi/2
    cart_ahead_tf.transform.rotation = tf2::toMsg(tf2::Quaternion(
        0, 0, std::sin(align_yaw / 2), std::cos(align_yaw / 2)));

    cart_ahead_static_tf_broadcaster_->sendTransform(cart_ahead_tf);

    RCLCPP_INFO(this->get_logger(),
                "Broadcasted static -%s- at (%.2f, %.2f) in Global frame",
                cart_ahead_frame_.c_str(), cart_ahead_tf_odom.point.x,
                cart_ahead_tf_odom.point.y);

    // Create the cart center frame
    geometry_msgs::msg::PointStamped cart_center_point;
    cart_center_point.header.frame_id = latest_scan_->header.frame_id;
    cart_center_point.point.x = cart_center_x;
    cart_center_point.point.y = cart_center_y;
    cart_center_point.point.z = 0.0;

    // Transform cart frame to global frame ("odom")
    geometry_msgs::msg::PointStamped cart_center_tf_odom;
    try {
      cart_center_tf_odom = tf_buffer_->transform(
          cart_center_point, odom_frame_, tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
      return false;
    }

    // Broadcast static TF from "odom" to "cart_frame"
    geometry_msgs::msg::TransformStamped cart_center_tf;
    cart_center_tf.header.stamp = this->get_clock()->now();
    cart_center_tf.header.frame_id = odom_frame_;
    // cart_center_tf.child_frame_id = "cart_center_frame";
    cart_center_tf.child_frame_id = cart_center_frame_;
    cart_center_tf.transform.translation.x = cart_center_tf_odom.point.x;
    cart_center_tf.transform.translation.y = cart_center_tf_odom.point.y;
    cart_center_tf.transform.translation.z = 0.0;

    // Orientation: perpendicular direction (for proper heading)
    float cart_center_yaw = std::atan2(center_entry_y, center_entry_x);

    // Rotate 90° right
    // cart_yaw = cart_yaw + M_PI_2; // pi/2
    cart_center_yaw = cart_center_yaw - M_PI_2; // pi/2
    cart_center_tf.transform.rotation = tf2::toMsg(tf2::Quaternion(
        0, 0, std::sin(cart_center_yaw / 2), std::cos(cart_center_yaw / 2)));

    cart_center_static_tf_broadcaster_->sendTransform(cart_center_tf);

    RCLCPP_INFO(this->get_logger(),
                "Broadcasted static -%s- at (%.2f, %.2f) in odom frame",
                cart_entry_frame_.c_str(), cart_center_tf_odom.point.x,
                cart_center_tf_odom.point.y);

    // Create the cart_entry_frame
    geometry_msgs::msg::PointStamped cart_entry_point;
    cart_entry_point.header.frame_id = latest_scan_->header.frame_id;
    cart_entry_point.point.x = center_entry_x;
    cart_entry_point.point.y = center_entry_y;
    cart_entry_point.point.z = 0.0;

    // Transform cart frame to global frame ("odom")
    geometry_msgs::msg::PointStamped cart_entry_tf_odom;
    try {
      cart_entry_tf_odom = tf_buffer_->transform(cart_entry_point, odom_frame_,
                                                 tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
      return false;
    }

    // Broadcast static TF from "odom" to "cart_entry_frame"
    geometry_msgs::msg::TransformStamped cart_entry_tf;
    cart_entry_tf.header.stamp = this->get_clock()->now();
    cart_entry_tf.header.frame_id = odom_frame_;
    cart_entry_tf.child_frame_id = cart_entry_frame_;
    cart_entry_tf.transform.translation.x = cart_entry_tf_odom.point.x;
    cart_entry_tf.transform.translation.y = cart_entry_tf_odom.point.y;
    cart_entry_tf.transform.translation.z = 0.0;

    // Orientation: perpendicular direction (for proper heading)
    float cart_yaw = std::atan2(center_entry_y, center_entry_x);

    // Rotate 90° right
    // cart_yaw = cart_yaw + M_PI_2; // pi/2
    cart_yaw = cart_yaw - M_PI_2; // pi/2
    cart_entry_tf.transform.rotation = tf2::toMsg(
        tf2::Quaternion(0, 0, std::sin(cart_yaw / 2), std::cos(cart_yaw / 2)));

    cart_ahead_static_tf_broadcaster_->sendTransform(cart_entry_tf);

    RCLCPP_INFO(this->get_logger(),
                "Broadcasted static -%s- at (%.2f, %.2f) in Global frame",
                cart_entry_frame_.c_str(), cart_entry_tf_odom.point.x,
                cart_entry_tf_odom.point.y);

    return true;
  }

  bool perform_final_approach() {

    std::string robot_frame = base_frame_;

    // move it to debug
    RCLCPP_INFO(get_logger(), "Starting Final Approach (Direct Control)");

    // STEP 1: Navigate to align position
    RCLCPP_INFO(get_logger(), "Step 1/3: Moving to align position...");

    if (!move_to_tf_frame(cart_ahead_frame_)) {
      RCLCPP_ERROR(get_logger(), "Failed to reach align position");
      stop_robot();
      return false;
    }

    RCLCPP_INFO(get_logger(), "Step 1 Complete: Reached align position!");

    // Small pause between waypoints
    // stop_robot();
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // STEP 2: Navigate to cart position
    RCLCPP_INFO(get_logger(), "Step 2/3: Moving to cart position...");

    if (!move_to_tf_frame(cart_entry_frame_)) {
      RCLCPP_ERROR(get_logger(), "Failed to reach cart position");
      stop_robot();
      return false;
    }

    RCLCPP_INFO(get_logger(), "Step 2 Complete: Reached cart position!");
    // stop_robot();

    RCLCPP_INFO(get_logger(), "Step 3/3: Moving to cart center position...");
    // move_forward(shelf_center_distance_, shelf_center_speed_);
    if (!move_to_tf_frame(cart_center_frame_)) {
      RCLCPP_ERROR(get_logger(), "Failed to reach cart position");
      stop_robot();
      return false;
    }

    RCLCPP_INFO(get_logger(), "Step 3 Complete: Reached cart position!");
    // stop_robot();

    shelf_attached_ = attach_shelf();

    RCLCPP_INFO(get_logger(), "Final Approach Complete");

    return true;
  }

  bool move_to_tf_frame(const std::string &frame_name) {

    // Get goal pose from TF
    double goal_x, goal_y, goal_yaw;
    if (!get_pose_from_tf(frame_name, goal_x, goal_y, goal_yaw)) {
      RCLCPP_INFO(get_logger(), "Failed to get goal pose from TF");
      return false;
    }

    RCLCPP_INFO(get_logger(),
                "Target: (%.2f, %.2f, %.2f°) | Current: (%.2f, %.2f, %.2f°)",
                goal_x, goal_y, goal_yaw * 180.0 / M_PI, current_x_, current_y_,
                current_yaw_ * 180.0 / M_PI);

    // Phase 1: Rotate to face the goal
    if (!rotate_to_face_goal(goal_x, goal_y)) {
      return false;
    }

    // Phase 2: Drive forward to goal position
    if (!drive_to_position(goal_x, goal_y)) {
      return false;
    }

    // Phase 3: Rotate to final orientation
    if (!rotate_to_angle(goal_yaw)) {
      return false;
    }

    return true;
  }

  bool get_robot_pose_from_tf(double &x, double &y, double &yaw) {
    try {
      auto transform = tf_buffer_->lookupTransform(odom_frame_, base_frame_,
                                                   tf2::TimePointZero);

      x = transform.transform.translation.x;
      y = transform.transform.translation.y;

      tf2::Quaternion q(
          transform.transform.rotation.x, transform.transform.rotation.y,
          transform.transform.rotation.z, transform.transform.rotation.w);

      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      return true;
    } catch (const tf2::TransformException &ex) {
      return false;
    }
  }

  bool get_pose_from_tf(const std::string &target_frame, double &x, double &y,
                        double &yaw) {
    try {
      auto transform = tf_buffer_->lookupTransform(
          odom_frame_, target_frame, tf2::TimePointZero,
          std::chrono::milliseconds(500));

      x = transform.transform.translation.x;
      y = transform.transform.translation.y;

      tf2::Quaternion q(
          transform.transform.rotation.x, transform.transform.rotation.y,
          transform.transform.rotation.z, transform.transform.rotation.w);

      tf2::Matrix3x3 m(q);
      double roll, pitch;
      m.getRPY(roll, pitch, yaw);

      // Debug logging
      RCLCPP_INFO(
          get_logger(), "TF Lookup: %s → %s: pos=(%.2f, %.2f), yaw=%.2f°",
          odom_frame_.c_str(), target_frame.c_str(), x, y, yaw * 180.0 / M_PI);

      return true;

    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "Failed to get TF for '%s': %s",
                   target_frame.c_str(), ex.what());
      return false;
    }
  }

  bool rotate_to_face_goal(double goal_x, double goal_y) {
    RCLCPP_INFO(get_logger(), "Phase 1: Rotating to face goal...");

    rclcpp::Rate rate(20); // 20 Hz control loop
    int timeout_count = 0;
    // const int max_timeout = 100; // 5 seconds at 20Hz
    const int max_timeout = 200; // 10 seconds at 20Hz

    while (rclcpp::ok()) {

      double robot_x, robot_y, robot_yaw;
      if (!get_robot_pose_from_tf(robot_x, robot_y, robot_yaw)) {
        RCLCPP_WARN(get_logger(), "Failed to get robot pose from TF");
        rate.sleep();
        continue;
      }

      // Calculate desired angle to goal
      double dx = goal_x - robot_x;
      double dy = goal_y - robot_y;
      double desired_yaw = atan2(dy, dx);

      // Calculate angle error
      double angle_error = normalize_angle(desired_yaw - robot_yaw);

      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "angle_error: (%.2f) | Current Yaw: (%.2f) → Goal Yaw: (%.2f)",
          angle_error, robot_yaw, desired_yaw);

      // Check if we're facing the goal
      if (fabs(angle_error) < angle_tolerance_) {
        stop_robot();
        RCLCPP_INFO(get_logger(), "Facing Target frame");
        return true;
      }

      // Send rotation command
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = copysign(angular_speed_, angle_error);
      cmd_vel_publisher_->publish(cmd);

      // Safety timeout
      if (++timeout_count > max_timeout) {
        RCLCPP_WARN(get_logger(), "Rotation timeout");
        stop_robot();
        return false;
      }

      rate.sleep();
    }

    return false;
  }

  bool drive_to_position(double goal_x, double goal_y) {
    RCLCPP_INFO(get_logger(), "Phase 2: Driving to position...");

    rclcpp::Rate rate(20); // 20 Hz control loop
    int timeout_count = 0;
    const int max_timeout = 200; // 10 seconds at 20Hz

    while (rclcpp::ok()) {
      // Get current robot position from TF
      double robot_x, robot_y, robot_yaw;
      if (!get_robot_pose_from_tf(robot_x, robot_y, robot_yaw)) {
        RCLCPP_WARN(get_logger(), "Failed to get robot pose from TF");
        rate.sleep();
        continue;
      }

      // Calculate distance to goal
      double dx = goal_x - robot_x;
      double dy = goal_y - robot_y;
      double distance = sqrt(dx * dx + dy * dy);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                           "Dist: %.2fm | Pos: (%.2f,%.2f) → Goal: (%.2f,%.2f)",
                           distance, robot_x, robot_y, goal_x, goal_y);

      // Check if we've reached the position
      if (distance < position_tolerance_) {
        stop_robot();
        RCLCPP_INFO(get_logger(), "Reached position");
        return true;
      }

      // Calculate heading error (to maintain direction)
      double desired_yaw = atan2(dy, dx);
      double angle_error = normalize_angle(desired_yaw - robot_yaw);

      // Send drive command with heading correction
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = linear_speed_;
      cmd.angular.z = 2.0 * angle_error; // Proportional heading correction

      // Limit angular velocity
      cmd.angular.z =
          std::max(-angular_speed_, std::min(angular_speed_, cmd.angular.z));

      cmd_vel_publisher_->publish(cmd);

      // Safety timeout
      if (++timeout_count > max_timeout) {
        RCLCPP_WARN(get_logger(), "Drive timeout");
        stop_robot();
        return false;
      }

      rate.sleep();
    }

    return false;
  }

  bool rotate_to_angle(double goal_yaw) {
    RCLCPP_INFO(get_logger(), "Phase 3: Rotating to final orientation...");

    rclcpp::Rate rate(20); // 20 Hz control loop
    int timeout_count = 0;
    const int max_timeout = 100; // 5 seconds at 20Hz

    while (rclcpp::ok()) {
      double robot_x, robot_y, robot_yaw;
      if (!get_robot_pose_from_tf(robot_x, robot_y, robot_yaw)) {
        RCLCPP_WARN(get_logger(), "Failed to get robot pose from TF");
        rate.sleep();
        continue;
      }

      // Calculate angle error
      double angle_error = normalize_angle(goal_yaw - robot_yaw);

      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "angle_error: (%.2f) | Current Yaw: (%.2f) → Goal Yaw: (%.2f)",
          angle_error, robot_yaw, goal_yaw);
      // Check if we're at the correct angle
      if (fabs(angle_error) < angle_tolerance_) {
        stop_robot();
        RCLCPP_INFO(get_logger(), "Target frames orientation achieved");
        return true;
      }

      // Send rotation command
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = copysign(angular_speed_, angle_error);
      cmd_vel_publisher_->publish(cmd);

      // Safety timeout
      if (++timeout_count > max_timeout) {
        RCLCPP_WARN(get_logger(), "Final rotation timeout");
        stop_robot();
        return false;
      }

      rate.sleep();
    }

    return false;
  }

  void move_forward(double distance, double speed) {
    if (speed <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "Speed must be positive!");
      return;
    }

    // Time calculation
    double duration = distance / speed;
    auto start = this->now();

    // 50 Hz loop
    rclcpp::Rate rate(50);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = speed;

    RCLCPP_INFO(this->get_logger(),
                "Moving forward: distance=%.2f m speed=%.2f m/s (%.2f seconds)",
                distance, speed, duration);

    while (rclcpp::ok()) {
      rclcpp::Time now = this->now();
      double elapsed = (now - start).seconds();

      if (elapsed >= duration) {
        RCLCPP_INFO(this->get_logger(), "Distance reached. Stopping.");
        break;
      }

      cmd_vel_publisher_->publish(cmd);
      rate.sleep();
    }

    // stop the robot
    geometry_msgs::msg::Twist stop_msg;
    cmd_vel_publisher_->publish(stop_msg);
  }

  void stop_robot() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_publisher_->publish(cmd);
  }

  double normalize_angle(double angle) {
    // Normalize angle to [-pi, pi]
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  bool attach_shelf() {
    // Lift the shelf
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    auto elevator_publisher =
        this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);
    rclcpp::sleep_for(
        std::chrono::milliseconds(1000)); // Let publisher initialize

    auto elevator_msg = std_msgs::msg::String();
    elevator_msg.data = "";

    // Publish message multiple times with small delays
    for (int i = 0; i < 3; i++) {
      elevator_publisher->publish(elevator_msg);
      RCLCPP_INFO(this->get_logger(), "Published elevator up message (%d/3)",
                  i + 1);
      rclcpp::sleep_for(
          std::chrono::milliseconds(500)); // 500ms delay between messages
    }
    rclcpp::sleep_for(
        std::chrono::milliseconds(5500)); // Let publisher initialize

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

// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);

//   auto node = std::make_shared<ApproachServiceServer>();

//   rclcpp::executors::MultiThreadedExecutor
//   executor(rclcpp::ExecutorOptions(),
//                                                     4); // 2 threads

//   executor.add_node(node);
//   executor.spin();

//   rclcpp::shutdown();
//   return 0;
// }