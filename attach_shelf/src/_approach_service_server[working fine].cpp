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
    align_static_tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    cart_static_tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cartframe_broadcasted_ = false;
    alignframe_broadcasted_ = false;
    legs_found = false;
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
      align_static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster>
      cart_static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  float intensities_threshold_ = 0.0f;
  float align_ahead_dist_;
  bool alignframe_broadcasted_;
  bool cartframe_broadcasted_;
  bool legs_found;
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
      legs_found = detect_and_broadcast_cart_frame();

      if (!legs_found) {
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
      response->complete = true; // ? doubtful what is the case scenario
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

    float mid_x = (x1 + x2) / 2.0f;
    float mid_y = (y1 + y2) / 2.0f;

    // `align_tf`
    // Line that is perpendicular to legs passing through cart frame for
    // alignment Direction vector (leg to leg) and length
    float dx = x2 - x1;
    float dy = y2 - y1;
    float length = std::sqrt(dx * dx + dy * dy);

    // Perpendicular unit vector
    float perp_x = -dy / length;
    float perp_y = dx / length;

    // Alignment correction coordinates: 10cm ahead
    // float offset = 0.10f; // 10cm
    float offset = align_ahead_dist_;
    float align_x = mid_x + offset * perp_x;
    float align_y = mid_y + offset * perp_y;

    // Create the alignment frame before cart frame
    geometry_msgs::msg::PointStamped align_point;
    align_point.header.frame_id = latest_scan_->header.frame_id;
    align_point.point.x = align_x;
    align_point.point.y = align_y;
    align_point.point.z = 0.0;

    // Transform to global frame ("odom")
    geometry_msgs::msg::PointStamped align_tf_odom;
    try {
      align_tf_odom = tf_buffer_->transform(align_point, odom_frame_,
                                            tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
      return false;
    }

    // Broadcast static TF from "odom" to "alignment_correction_frame"
    geometry_msgs::msg::TransformStamped align_tf;
    align_tf.header.stamp = this->get_clock()->now();
    // align_tf.header.frame_id = "map";
    align_tf.header.frame_id = odom_frame_;
    align_tf.child_frame_id = "alignment_correction_frame";
    align_tf.transform.translation.x = align_tf_odom.point.x;
    align_tf.transform.translation.y = align_tf_odom.point.y;
    align_tf.transform.translation.z = 0.0;

    // Orientation: perpendicular direction (for proper heading)
    // float align_yaw = std::atan2(perp_y, perp_x);
    float align_yaw = std::atan2(align_y, align_x);

    // Rotate 90° left
    // align_yaw = align_yaw + M_PI_2; // pi/2
    align_yaw = align_yaw - M_PI_2; // pi/2
    align_tf.transform.rotation = tf2::toMsg(tf2::Quaternion(
        0, 0, std::sin(align_yaw / 2), std::cos(align_yaw / 2)));

    align_static_tf_broadcaster_->sendTransform(align_tf);

    RCLCPP_INFO(
        this->get_logger(),
        "Broadcasted static -alignment_correction_frame- at (%.2f, %.2f) in "
        "odom frame",
        align_tf_odom.point.x, align_tf_odom.point.y);

    // Create the cart frame
    geometry_msgs::msg::PointStamped cart_point;
    cart_point.header.frame_id = latest_scan_->header.frame_id;
    cart_point.point.x = mid_x;
    cart_point.point.y = mid_y;
    cart_point.point.z = 0.0;

    // Transform cart frame to global frame ("odom")
    geometry_msgs::msg::PointStamped cart_tf_odom;
    try {
      cart_tf_odom = tf_buffer_->transform(cart_point, odom_frame_,
                                           tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
      return false;
    }

    // Broadcast static TF from "odom" to "cart_frame"
    geometry_msgs::msg::TransformStamped cart_tf;
    cart_tf.header.stamp = this->get_clock()->now();
    // cart_tf.header.frame_id = "map";
    cart_tf.header.frame_id = odom_frame_;
    cart_tf.child_frame_id = "cart_frame";
    cart_tf.transform.translation.x = cart_tf_odom.point.x;
    cart_tf.transform.translation.y = cart_tf_odom.point.y;
    cart_tf.transform.translation.z = 0.0;

    // Orientation: perpendicular direction (for proper heading)
    float cart_yaw = std::atan2(mid_y, mid_x);

    // Rotate 90° right
    // cart_yaw = cart_yaw + M_PI_2; // pi/2
    cart_yaw = cart_yaw - M_PI_2; // pi/2
    cart_tf.transform.rotation = tf2::toMsg(
        tf2::Quaternion(0, 0, std::sin(cart_yaw / 2), std::cos(cart_yaw / 2)));

    cart_static_tf_broadcaster_->sendTransform(cart_tf);

    RCLCPP_INFO(this->get_logger(),
                "Broadcasted static -cart_frame- at (%.2f, %.2f) in odom frame",
                cart_tf_odom.point.x, cart_tf_odom.point.y);

    return true;
  }

  //   bool perform_final_approach() {
  //     RCLCPP_INFO(
  //         this->get_logger(),
  //         "Starting perform_final_approach: beginning shelf approach
  //         sequence");

  //     std::string robot_frame = base_frame_;
  //     std::string align_frame = "alignment_correction_frame";
  //     std::string target_frame = "cart_frame";

  //     // Step 1: Move to alignment_correction_frame
  //     RCLCPP_INFO(this->get_logger(),
  //                 "Course correction to alignment_correction_frame");

  //     geometry_msgs::msg::TransformStamped align_transform;
  //     try {
  //       if (!tf_buffer_->canTransform(robot_frame, align_frame,
  //                                     tf2::TimePointZero,
  //                                     std::chrono::seconds(3))) {
  //         RCLCPP_ERROR(this->get_logger(),
  //                      "TF unavailable: Cannot transform from %s to %s",
  //                      robot_frame.c_str(), align_frame.c_str());
  //         return false;
  //       }

  //       align_transform = tf_buffer_->lookupTransform(robot_frame,
  //       align_frame,
  //                                                     tf2::TimePointZero);
  //       RCLCPP_DEBUG(this->get_logger(),
  //                    "Transform to alignment_correction_frame obtained");
  //     } catch (tf2::TransformException &ex) {
  //       RCLCPP_ERROR(this->get_logger(),
  //                    "TF2 exception aligning to correction frame: %s",
  //                    ex.what());
  //       return false;
  //     }

  //     double align_x = align_transform.transform.translation.x;
  //     double align_y = align_transform.transform.translation.y;

  //     RCLCPP_INFO(this->get_logger(),
  //                 "Alignment correction point (robot frame): x=%.3f, y=%.3f",
  //                 align_x, align_y);

  //     geometry_msgs::msg::Quaternion q = align_transform.transform.rotation;
  //     tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  //     double roll, pitch, align_yaw;
  //     tf2::Matrix3x3(quat).getRPY(roll, pitch, align_yaw);

  //     RCLCPP_DEBUG(this->get_logger(),
  //                  "Alignment frame orientation (yaw): %.3f rad", align_yaw);

  //     // Step 1a: Rotate to face alignment_correction_frame
  //     double angle_to_face = std::atan2(align_y, align_x);
  //     RCLCPP_INFO(this->get_logger(),
  //                 "Rotating to face alignment_correction_frame, angle: %.3f
  //                 rad", angle_to_face);

  //     if (std::abs(angle_to_face) > align_angle_threshold_) {
  //       auto twist_msg = geometry_msgs::msg::Twist();
  //       twist_msg.linear.x = 0.0;
  //       twist_msg.angular.z =
  //           (angle_to_face > 0) ? align_angular_speed_ :
  //           -align_angular_speed_;
  //       double rotation_time = std::abs(angle_to_face) /
  //       align_angular_speed_; RCLCPP_DEBUG(this->get_logger(),
  //                    "Starting rotation, estimated time: %.2fs",
  //                    rotation_time);
  //       auto start_time = this->now();
  //       while ((this->now() - start_time).seconds() < rotation_time) {
  //         cmd_vel_publisher_->publish(twist_msg);
  //         rclcpp::sleep_for(std::chrono::milliseconds(50));
  //       }
  //       twist_msg.angular.z = 0.0;
  //       cmd_vel_publisher_->publish(twist_msg);
  //       rclcpp::sleep_for(std::chrono::milliseconds(200));
  //       RCLCPP_DEBUG(this->get_logger(),
  //                    "Rotation to face alignment_correction_frame complete");
  //     }

  //     // Step 1b: Move forward to alignment_correction_frame
  //     double dist_to_align = std::sqrt(align_x * align_x + align_y *
  //     align_y); RCLCPP_INFO(
  //         this->get_logger(),
  //         "Moving forward to alignment_correction_frame (distance: %.2f m)",
  //         dist_to_align);

  //     auto twist_msg = geometry_msgs::msg::Twist();
  //     twist_msg.linear.x = 0.2;
  //     twist_msg.angular.z = 0.0;
  //     double move_time = dist_to_align / 0.2;
  //     auto start_time = this->now();

  //     while ((this->now() - start_time).seconds() < move_time) {
  //       cmd_vel_publisher_->publish(twist_msg);
  //       rclcpp::sleep_for(std::chrono::milliseconds(50));
  //     }

  //     rclcpp::sleep_for(std::chrono::milliseconds(200));
  //     twist_msg.linear.x = 0.0;
  //     cmd_vel_publisher_->publish(twist_msg);
  //     rclcpp::sleep_for(std::chrono::milliseconds(200));
  //     RCLCPP_DEBUG(this->get_logger(),
  //                  "Arrived at alignment_correction_frame position");

  //     // Step 2: Orient robot’s heading with alignment frame yaw
  //     double angle_to_alignment = align_yaw - current_yaw_;
  //     while (angle_to_alignment > M_PI)
  //       angle_to_alignment -= 2 * M_PI;
  //     while (angle_to_alignment < -M_PI)
  //       angle_to_alignment += 2 * M_PI;

  //     RCLCPP_INFO(this->get_logger(),
  //                 "Correcting heading: angle to alignment frame yaw: %.3f
  //                 rad", angle_to_alignment);

  //     if (std::abs(angle_to_alignment) > align_angle_threshold_) {
  //       twist_msg.linear.x = 0.0;
  //       twist_msg.angular.z = (angle_to_alignment > 0) ? align_angular_speed_
  //                                                      :
  //                                                      -align_angular_speed_;
  //       double rotation_time =
  //           std::abs(angle_to_alignment) / align_angular_speed_;
  //       RCLCPP_DEBUG(this->get_logger(),
  //                    "Starting orientation correction, estimated time:
  //                    %.2fs", rotation_time);
  //       auto start_time_align = this->now();

  //       while ((this->now() - start_time_align).seconds() < rotation_time) {
  //         cmd_vel_publisher_->publish(twist_msg);
  //         rclcpp::sleep_for(std::chrono::milliseconds(50));
  //       }

  //       rclcpp::sleep_for(std::chrono::milliseconds(200));
  //       twist_msg.angular.z = 0.0;
  //       cmd_vel_publisher_->publish(twist_msg);
  //       rclcpp::sleep_for(std::chrono::milliseconds(200));
  //       RCLCPP_DEBUG(this->get_logger(),
  //                    "Heading correction to alignment yaw complete");
  //     } else {
  //       RCLCPP_DEBUG(
  //           this->get_logger(),
  //           "Heading within threshold of alignment yaw, no correction
  //           needed.");
  //     }

  //     // Step 3: Move and align to cart_frame
  //     RCLCPP_INFO(this->get_logger(), "Approaching cart_frame");

  //     geometry_msgs::msg::TransformStamped cart_transform;
  //     try {
  //       if (!tf_buffer_->canTransform(robot_frame, target_frame,
  //                                     tf2::TimePointZero,
  //                                     std::chrono::seconds(3))) {
  //         RCLCPP_ERROR(this->get_logger(),
  //                      "TF unavailable: Cannot transform from %s to %s",
  //                      robot_frame.c_str(), target_frame.c_str());
  //         return false;
  //       }
  //       cart_transform = tf_buffer_->lookupTransform(robot_frame,
  //       target_frame,
  //                                                    tf2::TimePointZero);
  //       RCLCPP_DEBUG(this->get_logger(), "Transform to cart_frame obtained");
  //     } catch (tf2::TransformException &ex) {
  //       RCLCPP_ERROR(this->get_logger(),
  //                    "TF2 exception approaching cart_frame: %s", ex.what());
  //       return false;
  //     }
  //     double cart_x = cart_transform.transform.translation.x;
  //     double cart_y = cart_transform.transform.translation.y;
  //     double angle_to_cart = std::atan2(cart_y, cart_x);

  //     RCLCPP_INFO(this->get_logger(),
  //                 "Rotating to face cart_frame: angle %.3f rad",
  //                 angle_to_cart);

  //     if (std::abs(angle_to_cart) > 0.05) {
  //       twist_msg.linear.x = 0.0;
  //       twist_msg.angular.z = (angle_to_cart > 0) ? 0.3 : -0.3;
  //       double rot_time = std::abs(angle_to_cart) / 0.3;
  //       RCLCPP_DEBUG(this->get_logger(),
  //                    "Starting rotation, estimated time: %.2fs", rot_time);
  //       auto start_time = this->now();

  //       while ((this->now() - start_time).seconds() < rot_time) {
  //         cmd_vel_publisher_->publish(twist_msg);
  //         rclcpp::sleep_for(std::chrono::milliseconds(50));
  //       }

  //       rclcpp::sleep_for(std::chrono::milliseconds(200));
  //       twist_msg.angular.z = 0.0;
  //       cmd_vel_publisher_->publish(twist_msg);
  //       rclcpp::sleep_for(std::chrono::milliseconds(200));
  //       RCLCPP_DEBUG(this->get_logger(), "Rotation to face cart_frame
  //       complete");
  //     }

  //     double dist_to_cart = std::sqrt(cart_x * cart_x + cart_y * cart_y);
  //     RCLCPP_INFO(this->get_logger(), "Moving to cart_frame (distance: %.2f
  //     m)",
  //                 dist_to_cart);

  //     move_forward(dist_to_cart, 0.2);

  //     RCLCPP_INFO(
  //         this->get_logger(),
  //         "Reached cart_frame position - now proceeding to final approach");
  //     rclcpp::sleep_for(std::chrono::milliseconds(500));

  //     move_forward(0.3, 0.1);
  //     RCLCPP_INFO(this->get_logger(), "Robot positioned under shelf");

  //     return true;
  //   }

  bool perform_final_approach() {

    std::string robot_frame = base_frame_;
    std::string align_frame = "alignment_correction_frame";
    std::string target_frame = "cart_frame";

    // move it to debug
    RCLCPP_INFO(get_logger(), "Starting Final Approach (Direct Control)");

    // STEP 1: Navigate to align position
    RCLCPP_INFO(get_logger(), "Step 1/3: Moving to align position...");

    // if (!move_to_tf_frame("align_static_tf")) {
    if (!move_to_tf_frame(align_frame)) {
      RCLCPP_ERROR(get_logger(), "Failed to reach align position");
      stop_robot();
      return false;
    }

    RCLCPP_INFO(get_logger(), "Step 1 Complete: Reached align position!");

    // Small pause between waypoints
    stop_robot();
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // STEP 2: Navigate to cart position
    RCLCPP_INFO(get_logger(), "Step 2/3: Moving to cart position...");

    // if (!move_to_tf_frame("cart_static_tf")) {
    if (!move_to_tf_frame(target_frame)) {
      RCLCPP_ERROR(get_logger(), "Failed to reach cart position");
      stop_robot();
      return false;
    }

    RCLCPP_INFO(get_logger(), "Step 2 Complete: Reached cart position!");
    stop_robot();
    

    RCLCPP_INFO(get_logger(), "Step 3/3: Moving to cart center position...");
    move_forward(shelf_center_distance_, shelf_center_speed_);
    
    RCLCPP_INFO(get_logger(), "Step 3 Complete: Reached cart position!");
    stop_robot();

    RCLCPP_INFO(get_logger(), "Final Approach Complete");
    return true;
  }

  bool move_to_tf_frame(const std::string &frame_name) {
    // Wait for odometry
    // if (!wait_for_odometry()) {
    //   RCLCPP_ERROR(get_logger(), "No odometry received!");
    //   return false;
    // }

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
      auto transform = tf_buffer_->lookupTransform(odom_frame_, // odom
                                                   base_frame_, // base_link
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
      //   auto transform =
      //       tf_buffer_->lookupTransform("map", target_frame,
      //       tf2::TimePointZero,
      //                                   std::chrono::milliseconds(500));
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
      //   double dx = goal_x - current_x_;
      //   double dy = goal_y - current_y_;
      double dx = goal_x - robot_x;
      double dy = goal_y - robot_y;
      double desired_yaw = atan2(dy, dx);

      // Calculate angle error
      //   double angle_error = normalize_angle(desired_yaw - current_yaw_);
      double angle_error = normalize_angle(desired_yaw - robot_yaw);

      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "angle_error: (%.2f) | Current Yaw: (%.2f) → Goal Yaw: (%.2f)",
          angle_error, robot_yaw, desired_yaw);

      // Check if we're facing the goal
      if (fabs(angle_error) < angle_tolerance_) {
        stop_robot();
        RCLCPP_INFO(get_logger(), "Facing goal|||||||             |||||||||");
        return true;
      }

      // Send rotation command
      //   RCLCPP_INFO(get_logger(), "--->Rotating");
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = copysign(angular_speed_, angle_error);
      cmd_vel_publisher_->publish(cmd);

      // Safety timeout
      if (++timeout_count > max_timeout) {
        RCLCPP_WARN(get_logger(), "Rotation timeout");
        stop_robot();
        return false;
      }

      // rclcpp::spin_some(this->get_node_base_interface());
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
      //   double dx = goal_x - current_x_;
      //   double dy = goal_y - current_y_;
      double dx = goal_x - robot_x;
      double dy = goal_y - robot_y;
      double distance = sqrt(dx * dx + dy * dy);

      //   RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      //                        "Pos: (%.2f,%.2f) → Goal: (%.2f,%.2f) | Dist:
      //                        %.2fm", current_x_, current_y_, goal_x, goal_y,
      //                        distance);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                           "Dist: %.2fm | Pos: (%.2f,%.2f) → Goal: (%.2f,%.2f)",
                           distance, robot_x, robot_y, goal_x, goal_y);

      // Check if we've reached the position
      if (distance < position_tolerance_) {
        stop_robot();
        RCLCPP_INFO(get_logger(), "Reached position|||||^^^^||||||||");
        return true;
      }

      // Calculate heading error (to maintain direction)
      double desired_yaw = atan2(dy, dx);
      //   double angle_error = normalize_angle(desired_yaw - current_yaw_);
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

      // rclcpp::spin_some(this->get_node_base_interface());
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
      //   double angle_error = normalize_angle(goal_yaw - current_yaw_);
      double angle_error = normalize_angle(goal_yaw - robot_yaw);

      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "angle_error: (%.2f) | Current Yaw: (%.2f) → Goal Yaw: (%.2f)",
          angle_error, robot_yaw, goal_yaw);
      // Check if we're at the correct angle
      if (fabs(angle_error) < angle_tolerance_) {
        stop_robot();
        RCLCPP_INFO(
            get_logger(),
            "Final orientation achieved|||||||||||*********|||||||||||S");
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

      // rclcpp::spin_some(this->get_node_base_interface());
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

  bool wait_for_odometry() {
    rclcpp::Rate rate(10);
    int timeout = 50; // 5 seconds

    while (!odom_received_ && timeout > 0 && rclcpp::ok()) {
      rate.sleep();
      timeout--;
    }

    return odom_received_;
  }

  double normalize_angle(double angle) {
    // Normalize angle to [-pi, pi]
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
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