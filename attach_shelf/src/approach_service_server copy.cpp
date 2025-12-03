#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "memory"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
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

// DYNAMIC INTENSITY THRESHOLD

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer() : Node("approach_service_server") {

    // Read parameters from `robot_config` pkg
    this->declare_parameter<std::string>("odom_frame", "");
    this->declare_parameter<std::string>("base_frame", "");
    this->declare_parameter<std::string>("cmd_vel_topic", "");

    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("cmd_vel_topic", cmd_vel_topic_);

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
  bool alignframe_broadcasted_;
  bool cartframe_broadcasted_;
  bool legs_found;
  double current_yaw_;
  double front_distance_;
  float align_angle_threshold_ = 0.5;
  double align_angular_speed_;
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
      response->complete = true;        // ? doubtful what is the case scenario
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
    float offset = 0.10f; // 10cm
    float align_x = mid_x + offset * perp_x;
    float align_y = mid_y + offset * perp_y;

    // Create the cart midpoint in laser frame
    geometry_msgs::msg::PointStamped align_tf_point;
    align_tf_point.header.frame_id = latest_scan_->header.frame_id;
    align_tf_point.point.x = align_x;
    align_tf_point.point.y = align_y;
    align_tf_point.point.z = 0.0;

    // Transform to global frame ("odom")
    geometry_msgs::msg::PointStamped align_tf_odom;
    try {
      align_tf_odom = tf_buffer_->transform(align_tf_odom, odom_frame_,
                                            tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
      return false;
    }

    // Broadcast static TF from "odom" to "alignment_correction_frame"
    geometry_msgs::msg::TransformStamped align_tf;
    align_tf.header.stamp = this->get_clock()->now();
    align_tf.header.frame_id =
        odom_frame_; // TODO: what if the parent is cart_frame
    align_tf.child_frame_id = "alignment_correction_frame";
    align_tf.transform.translation.x = align_tf_odom.point.x;
    align_tf.transform.translation.y = align_tf_odom.point.y;
    align_tf.transform.translation.z = 0.0;

    // Orientation: perpendicular direction (for proper heading)
    float yaw = std::atan2(perp_y, perp_x);
    align_tf.transform.rotation =
        tf2::toMsg(tf2::Quaternion(0, 0, std::sin(yaw / 2), std::cos(yaw / 2)));

    align_static_tf_broadcaster_->sendTransform(align_tf);

    RCLCPP_INFO(this->get_logger(),
                "Broadcasted static -alignment_correction_frame- at (%.2f, %.2f) in "
                "odom frame",
                align_tf_odom.point.x, align_tf_odom.point.y);

    // Create the cart midpoint in laser frame
    geometry_msgs::msg::PointStamped point_in_laser;
    point_in_laser.header.frame_id = latest_scan_->header.frame_id;
    point_in_laser.point.x = mid_x;
    point_in_laser.point.y = mid_y;
    point_in_laser.point.z = 0.0;

    // Transform to global frame ("odom")
    geometry_msgs::msg::PointStamped point_in_odom;
    try {
      point_in_odom = tf_buffer_->transform(point_in_laser, odom_frame_,
                                            tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
      return false;
    }

    // Broadcast static TF from "odom" to "cart_frame"
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = odom_frame_;
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
                "Broadcasted static -cart_frame- at (%.2f, %.2f) in odom frame",
                point_in_odom.point.x, point_in_odom.point.y);

    return true;
  }

  bool perform_final_approach() {
    RCLCPP_INFO(this->get_logger(), "Starting to approach shelf");

    std::string robot_frame = base_frame_;
    std::string align_frame = "alignment_correction_frame";
    std::string target_frame = "cart_frame";

    // --- Step 1: Move to alignment_correction_frame ---

    RCLCPP_INFO(this->get_logger(), "Preparing to align towards align_frame");
    geometry_msgs::msg::TransformStamped align_transform;
    try {
      if (!tf_buffer_->canTransform(robot_frame, align_frame,
                                    tf2::TimePointZero,
                                    std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(),
                     "Transform from %s to %s not available",
                     robot_frame.c_str(), align_frame.c_str());
        return false;
      }
      align_transform = tf_buffer_->lookupTransform(robot_frame, align_frame,
                                                    tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "TF2 exception: %s", ex.what());
      return false;
    }

    double align_x = align_transform.transform.translation.x;
    double align_y = align_transform.transform.translation.y;

    // rotate the robot to match the alignment frame's yaw orientation
    geometry_msgs::msg::Quaternion q = align_transform.transform.rotation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, align_yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, align_yaw);

    // -- Rotate robot to face 'alignment_correction_frame' --
    double angle_to_face = std::atan2(align_y, align_x);
    if (std::abs(angle_to_face) > align_angle_threshold_) {
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z =
          (angle_to_face > 0) ? align_angular_speed_ : -align_angular_speed_;
      double rotation_time = std::abs(angle_to_face) / align_angular_speed_;
      auto start_time = this->now();
      while ((this->now() - start_time).seconds() < rotation_time) {
        cmd_vel_publisher_->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
      }
      twist_msg.angular.z = 0.0;
      cmd_vel_publisher_->publish(twist_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    // -- Move forward to alignment_correction_frame --
    double dist_to_align = std::sqrt(align_x * align_x + align_y * align_y);
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.2;
    twist_msg.angular.z = 0.0;
    double move_time = dist_to_align / 0.2;
    auto start_time = this->now();
    while ((this->now() - start_time).seconds() < move_time) {
      cmd_vel_publisher_->publish(twist_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    twist_msg.linear.x = 0.0;
    cmd_vel_publisher_->publish(twist_msg);

    // Step 2: Re-orient robot heading to match alignment frame yaw exactly
    double angle_to_alignment = align_yaw - current_yaw_;

    // Normalize angle_to_alignment between -pi and pi
    while (angle_to_alignment > M_PI)
      angle_to_alignment -= 2 * M_PI;
    while (angle_to_alignment < -M_PI)
      angle_to_alignment += 2 * M_PI;

    if (std::abs(angle_to_alignment) > align_angle_threshold_) {
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = (angle_to_alignment > 0) ? align_angular_speed_
                                                     : -align_angular_speed_;
      double rotation_time =
          std::abs(angle_to_alignment) / align_angular_speed_;
      auto start_time_align = this->now();
      while ((this->now() - start_time_align).seconds() < rotation_time) {
        cmd_vel_publisher_->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
      }
      twist_msg.angular.z = 0.0;
      cmd_vel_publisher_->publish(twist_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    // -- Step 3: Now at alignment_correction_frame. Re-align robot's heading to
    // cart_frame --

    geometry_msgs::msg::TransformStamped cart_transform;
    try {
      if (!tf_buffer_->canTransform(robot_frame, target_frame,
                                    tf2::TimePointZero,
                                    std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(),
                     "Transform from %s to %s not available",
                     robot_frame.c_str(), target_frame.c_str());
        return false;
      }
      cart_transform = tf_buffer_->lookupTransform(robot_frame, target_frame,
                                                   tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "TF2 exception: %s", ex.what());
      return false;
    }
    double cart_x = cart_transform.transform.translation.x;
    double cart_y = cart_transform.transform.translation.y;
    double angle_to_cart = std::atan2(cart_y, cart_x);

    // -- Rotate to face cart_frame again if needed --
    if (std::abs(angle_to_cart) > 0.05) {
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = (angle_to_cart > 0) ? 0.3 : -0.3;
      double rot_time = std::abs(angle_to_cart) / 0.3;
      auto start_time = this->now();
      while ((this->now() - start_time).seconds() < rot_time) {
        cmd_vel_publisher_->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
      }
      twist_msg.angular.z = 0.0;
      cmd_vel_publisher_->publish(twist_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    // -- Move to cart_frame --
    double dist_to_cart = std::sqrt(cart_x * cart_x + cart_y * cart_y);
    // twist_msg.linear.x = 0.2;
    // twist_msg.angular.z = 0.0;
    // double move_time_cart = dist_to_cart / 0.2;
    // auto start_time_cart = this->now();
    // while ((this->now() - start_time_cart).seconds() < move_time_cart) {
    //   cmd_vel_publisher_->publish(twist_msg);
    //   rclcpp::sleep_for(std::chrono::milliseconds(50));
    // }
    // twist_msg.linear.x = 0.0;
    // cmd_vel_publisher_->publish(twist_msg);

    move_forward(dist_to_cart, 0.2);
    RCLCPP_INFO(this->get_logger(), "Reached cart_frame position");

    std::chrono::milliseconds(500);

    move_forward(0.3, 0.1);
    RCLCPP_INFO(this->get_logger(), "Robot positioned under shelf");
    return true;
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
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachServiceServer>());
  rclcpp::shutdown();
  return 0;
}
