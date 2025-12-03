#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <algorithm>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

using namespace std::chrono_literals;

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer() : Node("approach_service_server") {

    // Declare parameters
    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<bool>("debug_restrict_move", false);
    this->declare_parameter<bool>("debug_restrict_attach_shelf", false);
    this->declare_parameter<std::string>("global_frame", "map");
    this->declare_parameter<std::string>("odom_frame");
    this->declare_parameter<std::string>("base_frame", "robot_base_footprint");
    this->declare_parameter<std::string>("cmd_vel_topic", "/robot/cmd_vel");
    this->declare_parameter<float>("intensity_thres_fact");
    this->declare_parameter<float>("front_side_band");
    this->declare_parameter<bool>("cart_ref_off");
    this->declare_parameter<bool>("flip_cart_frames");
    this->declare_parameter<float>("align_ahead_dist");
    this->declare_parameter<float>("align_cart_center_dist");
    this->declare_parameter<double>("position_tolerance");
    this->declare_parameter<double>("angle_tolerance");
    this->declare_parameter<double>("angular_speed");
    this->declare_parameter<double>("linear_speed");
    this->declare_parameter<double>("shelf_center_distance");
    this->declare_parameter<double>("shelf_center_speed");

    // Get parameters
    this->get_parameter("debug", debug_);
    this->get_parameter("debug_restrict_move", debug_restrict_move_);
    this->get_parameter("debug_restrict_attach_shelf",
                        debug_restrict_attach_shelf_);
    this->get_parameter("global_frame", global_frame_);
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
    this->get_parameter("intensity_thres_fact", intensity_thres_fact_);
    this->get_parameter("front_side_band", front_side_band_);
    this->get_parameter("align_ahead_dist", align_ahead_dist_);
    this->get_parameter("cart_ref_off", cart_ref_off_);
    this->get_parameter("flip_cart_frames", flip_cart_frames_);
    this->get_parameter("align_cart_center_dist", align_cart_center_dist_);
    this->get_parameter("position_tolerance", position_tolerance_);
    this->get_parameter("angle_tolerance", angle_tolerance_);
    this->get_parameter("angular_speed", angular_speed_);
    this->get_parameter("linear_speed", linear_speed_);
    this->get_parameter("shelf_center_distance", shelf_center_distance_);
    this->get_parameter("shelf_center_speed", shelf_center_speed_);

    RCLCPP_INFO(this->get_logger(), "Debugging: %s", debug_ ? "ON" : "OFF");
    RCLCPP_INFO(this->get_logger(), "Frames: global=%s, base=%s",
                global_frame_.c_str(), base_frame_.c_str());

    // Create service
    approach_shelf_service_ =
        this->create_service<attach_shelf::srv::GoToLoading>(
            "/approach_shelf",
            std::bind(&ApproachServiceServer::handle_approach_shelf, this,
                      std::placeholders::_1, std::placeholders::_2));

    // Create subscriptions
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ApproachServiceServer::laser_callback, this,
                  std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/diffbot_base_controller/odom", 10,
        std::bind(&ApproachServiceServer::odom_callback, this,
                  std::placeholders::_1));

    // Create publishers
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    // Create TF objects
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    static_tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "Approach Shelf Service Ready");
  }

private:
  // ROS2 objects
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr
      approach_shelf_service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // Parameters
  bool debug_;
  bool debug_restrict_move_;
  bool debug_restrict_attach_shelf_;
  std::string global_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string cmd_vel_topic_;
  float intensity_thres_fact_;
  float front_side_band_;
  bool cart_ref_off_;
  bool flip_cart_frames_;
  float align_ahead_dist_;
  float align_cart_center_dist_;
  double position_tolerance_;
  double angle_tolerance_;
  double angular_speed_;
  double linear_speed_;
  double shelf_center_distance_;
  double shelf_center_speed_;

  // Frame names
  const std::string cart_entry_frame_ = "cart_entry_frame";
  const std::string cart_ahead_frame_ = "cart_align_frame";
  const std::string cart_center_frame_ = "cart_center_frame";

  // State variables
  bool cart_frames_created_ = false;
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  struct Coord {
    float x;
    float y;
  };

  struct Positions {
    Coord entry;
    Coord align;
    Coord center;
  };
  Positions cart_pose;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

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

    RCLCPP_INFO(this->get_logger(), "Approach shelf requested: %s",
                request->attach_to_shelf ? "YES" : "NO");

    if (!request->attach_to_shelf) {
      response->complete = false;
      return;
    }

    if (!latest_scan_) {
      RCLCPP_WARN(this->get_logger(), "No laser scan data available");
      response->complete = false;
      return;
    }

    // Detect cart and create static frames
    if (!cart_frames_created_) {
      RCLCPP_INFO(this->get_logger(), "Detecting cart and creating frames...");

      // bool success = detect_and_create_cart_frames();
      bool found_legs = detect_legs();
      if (!found_legs) {
        // retry leg detection
        for (int i = 0; i < 4; ++i) {
          move_forward_small_distance();
          found_legs = detect_legs();

          if (found_legs) // exit early if found
            break;
        }
      }
    }

    cart_frames_created_ = create_static_cart_frames(cart_pose);
    if (!cart_frames_created_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create cart frames");
      response->complete = false;
      return;
    }

    //   cart_frames_created_ = true;
    RCLCPP_INFO(this->get_logger(), "✅ Cart frames created successfully");

    // Validate TF tree consistency once
    validate_tf_tree_consistency();
    //   }

    // Perform approach
    if (request->attach_to_shelf) {
      bool success = perform_final_approach();
      response->complete = success;
    } else {
      response->complete = true;
    }
  }

  // ============================================================================
  // CART DETECTION AND FRAME CREATION
  //   bool detect_cart_legs_with_retry(int max_retries = 3,
  //                                    float retry_step_distance = 0.05) {
  //     for (int attempt = 1; attempt <= max_retries; attempt++) {
  //       bool legs_found = detect_and_create_cart_frames();
  //       if (legs_found) {
  //         return true;
  //       }
  //       if (attempt < max_retries) {
  //         RCLCPP_WARN(get_logger(),
  //                     "⚠️  Attempt %d/%d failed - legs not found",
  //                     attempt, max_retries);
  //         RCLCPP_INFO(get_logger(), "Moving forward %.3fm before retry...",
  //                     retry_step_distance);

  //         if (!move_forward_small_distance(retry_step_distance)) {
  //           RCLCPP_ERROR(get_logger(), "Failed to move forward for retry");
  //           // Continue anyway - maybe detection will work from current
  //           position
  //         }

  //         // Small delay before retry
  //         rclcpp::sleep_for(std::chrono::milliseconds(300));
  //       }
  //     }
  //     return false;
  //   }

  bool detect_legs() {
    // Find shelf legs
    const std::vector<float> &intensity_data = latest_scan_->intensities;
    const std::vector<float> &ranges = latest_scan_->ranges;

    if (intensity_data.empty()) {
      RCLCPP_WARN(this->get_logger(), "No intensity data");
      return false;
    }

    // Calculate threshold
    float max_intensity =
        *std::max_element(intensity_data.begin(), intensity_data.end());
    float threshold = max_intensity * intensity_thres_fact_;

    RCLCPP_INFO(this->get_logger(), "max intensity found to be : %f",
                max_intensity);
    RCLCPP_INFO(this->get_logger(), "Intensity threshold: %.2f", threshold);

    // Find legs
    struct ShelfLeg {
      size_t start, end;
    };

    std::vector<ShelfLeg> legs;
    auto iter = intensity_data.begin();

    while (iter != intensity_data.end() && legs.size() < 2) {
      iter = std::find_if(iter, intensity_data.end(),
                          [threshold](float v) { return v >= threshold; });
      if (iter == intensity_data.end())
        break;

      size_t start = std::distance(intensity_data.begin(), iter);

      iter = std::find_if(iter, intensity_data.end(),
                          [threshold](float v) { return v < threshold; });
      size_t end = (iter == intensity_data.end())
                       ? intensity_data.size() - 1
                       : std::distance(intensity_data.begin(), iter) - 1;

      legs.push_back({start, end});
      RCLCPP_INFO(this->get_logger(), "Leg %zu: [%zu, %zu]", legs.size(), start,
                  end);
    }

    if (legs.size() != 2) {
      RCLCPP_WARN(this->get_logger(), "Found %zu legs, need 2", legs.size());
      return false;
    }

    // Calculate leg positions in laser frame
    size_t left_idx = (legs[0].start + legs[0].end) / 2;
    size_t right_idx = (legs[1].start + legs[1].end) / 2;

    float angle_left =
        latest_scan_->angle_min + left_idx * latest_scan_->angle_increment;
    float angle_right =
        latest_scan_->angle_min + right_idx * latest_scan_->angle_increment;

    float x1 = ranges[left_idx] * std::cos(angle_left);
    float y1 = ranges[left_idx] * std::sin(angle_left);
    float x2 = ranges[right_idx] * std::cos(angle_right);
    float y2 = ranges[right_idx] * std::sin(angle_right);

    // Calculate entry point and perpendicular vector
    cart_pose.entry.x = (x1 + x2) / 2.0f;
    cart_pose.entry.y = (y1 + y2) / 2.0f;

    float dx = x2 - x1;
    float dy = y2 - y1;
    float length = std::sqrt(dx * dx + dy * dy);
    float perp_x = -dy / length;
    float perp_y = dx / length;

    // Calculate frame positions in laser frame
    cart_pose.align.x = cart_pose.entry.x + align_ahead_dist_ * perp_x;
    cart_pose.align.y = cart_pose.entry.y + align_ahead_dist_ * perp_y;
    cart_pose.center.x = cart_pose.entry.x - align_cart_center_dist_ * perp_x;
    cart_pose.center.y = cart_pose.entry.y - align_cart_center_dist_ * perp_y;

    if (debug_) {
      RCLCPP_INFO(this->get_logger(), "=== Geometry Analysis ===");
      RCLCPP_INFO(this->get_logger(), "Leg 1: (%.2f, %.2f)", x1, y1);
      RCLCPP_INFO(this->get_logger(), "Leg 2: (%.2f, %.2f)", x2, y2);
      RCLCPP_INFO(this->get_logger(), "Entry: (%.2f, %.2f)", cart_pose.entry.x,
                  cart_pose.entry.y);
      RCLCPP_INFO(this->get_logger(), "Perpendicular: (%.3f, %.3f)", perp_x,
                  perp_y);
    }
    if (legs.size() == 2) {
      return true;
    }
    return false;
  }

  bool create_static_cart_frames(const Positions &cart) {

    const std::string ref_frame = "robot_cart_laser";
    bool use_reference = false;
    geometry_msgs::msg::TransformStamped ref_tf;

    // Check if robot_cart_laser is available
    if (!cart_ref_off_) {
      try {
        ref_tf = tf_buffer_->lookupTransform(global_frame_, ref_frame,
                                             tf2::TimePointZero,
                                             tf2::durationFromSec(1.0));

        use_reference = true;
        RCLCPP_INFO(this->get_logger(),
                    "✅ Also, found %s frame, cloning position & orientation",
                    ref_frame.c_str());

      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "⚠️  %s not available: %s",
                    ref_frame.c_str(), ex.what());
        RCLCPP_INFO(this->get_logger(),
                    "Using fallback: transforming laser coordinates to %s",
                    global_frame_.c_str());
      }
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "FORCING cart frame creating to use legs");
    }
    std::vector<geometry_msgs::msg::TransformStamped> static_transforms;

    if (use_reference) {
      tf2::Transform T_ref;
      tf2::fromMsg(ref_tf.transform, T_ref);

      // ENTRY frame: Clone reference frame
      geometry_msgs::msg::TransformStamped entry_tf;
      entry_tf.header.stamp = this->now();
      entry_tf.header.frame_id = global_frame_; // map
      entry_tf.child_frame_id = cart_entry_frame_;
      entry_tf.transform = tf2::toMsg(T_ref);

      ////////////////// in simulation
      // ALIGN frame: Offset forward (in reference frame's X axis)
      //   tf2::Transform T_align =
      //       T_ref * tf2::Transform(tf2::Quaternion::getIdentity(),
      //                              tf2::Vector3(align_ahead_dist_, 0.0,
      //                              0.0));

      ////////////////// in real
      //   tf2::Transform T_align =
      //       T_ref * tf2::Transform(tf2::Quaternion::getIdentity(),
      //                              tf2::Vector3(-align_ahead_dist_, 0.0,
      //                              0.0));

      ////////////////// combined
      tf2::Transform T_align =
          T_ref *
          tf2::Transform(tf2::Quaternion::getIdentity(),
                         tf2::Vector3(align_ahead_dist_ *
                                          (flip_cart_frames_ ? -1.0f : 1.0f),
                                      0.0, 0.0));

      geometry_msgs::msg::TransformStamped align_tf;
      align_tf.header.stamp = this->now();
      align_tf.header.frame_id = global_frame_; // map
      align_tf.child_frame_id = cart_ahead_frame_;
      align_tf.transform = tf2::toMsg(T_align);

      ////////////////// in simulation
      // CENTER frame: Offset backward (in reference frame's X axis)
      //   tf2::Transform T_center =
      //       T_ref *
      //       tf2::Transform(tf2::Quaternion::getIdentity(),
      //                      tf2::Vector3(-align_cart_center_dist_, 0.0, 0.0));

      ////////////////// in real
      //   tf2::Transform T_center =
      //       T_ref *
      //       tf2::Transform(tf2::Quaternion::getIdentity(),
      //                      tf2::Vector3(align_cart_center_dist_, 0.0, 0.0));

      ////////////////// combined
      tf2::Transform T_center =
          T_ref *
          tf2::Transform(tf2::Quaternion::getIdentity(),
                         tf2::Vector3(-align_cart_center_dist_ *
                                          (flip_cart_frames_ ? -1.0f : 1.0f),
                                      0.0, 0.0));

      geometry_msgs::msg::TransformStamped center_tf;
      center_tf.header.stamp = this->now();
      center_tf.header.frame_id = global_frame_; // map
      center_tf.child_frame_id = cart_center_frame_;
      center_tf.transform = tf2::toMsg(T_center);

      static_transforms = {entry_tf, align_tf, center_tf};

      RCLCPP_INFO(this->get_logger(), "Created frames by cloning %s",
                  ref_frame.c_str());

    } else {
      // Creating from legs coordinates
      std::string laser_frame = latest_scan_->header.frame_id;

      try {
        // Transform entry point
        geometry_msgs::msg::PointStamped entry_pt;
        entry_pt.header.frame_id = laser_frame;
        entry_pt.header.stamp = latest_scan_->header.stamp;
        entry_pt.point.x = cart.entry.x;
        entry_pt.point.y = cart.entry.y;
        entry_pt.point.z = 0.0;

        auto entry_map = tf_buffer_->transform(entry_pt, global_frame_,
                                               tf2::durationFromSec(1.0));

        // Transform ahead point
        geometry_msgs::msg::PointStamped ahead_pt;
        ahead_pt.header.frame_id = laser_frame;
        ahead_pt.header.stamp = latest_scan_->header.stamp;
        ahead_pt.point.x = cart.align.x;
        ahead_pt.point.y = cart.align.y;
        ahead_pt.point.z = 0.0;

        auto ahead_map = tf_buffer_->transform(ahead_pt, global_frame_,
                                               tf2::durationFromSec(1.0));

        // Transform center point
        geometry_msgs::msg::PointStamped center_pt;
        center_pt.header.frame_id = laser_frame;
        center_pt.header.stamp = latest_scan_->header.stamp;
        center_pt.point.x = cart.center.x;
        center_pt.point.y = cart.center.y;
        center_pt.point.z = 0.0;

        auto center_map = tf_buffer_->transform(center_pt, global_frame_,
                                                tf2::durationFromSec(1.0));

        // Create orientation (facing into shelf, -90 degrees)
        tf2::Quaternion q;
        q.setRPY(0, 0, -M_PI / 2.0);
        geometry_msgs::msg::Quaternion orientation = tf2::toMsg(q);

        // ENTRY frame
        geometry_msgs::msg::TransformStamped entry_tf;
        entry_tf.header.stamp = this->now();
        entry_tf.header.frame_id = global_frame_;
        entry_tf.child_frame_id = cart_entry_frame_;
        entry_tf.transform.translation.x = entry_map.point.x;
        entry_tf.transform.translation.y = entry_map.point.y;
        entry_tf.transform.translation.z = 0.0;
        entry_tf.transform.rotation = orientation;

        // ALIGN frame
        geometry_msgs::msg::TransformStamped align_tf;
        align_tf.header.stamp = this->now();
        align_tf.header.frame_id = global_frame_;
        align_tf.child_frame_id = cart_ahead_frame_;
        align_tf.transform.translation.x = ahead_map.point.x;
        align_tf.transform.translation.y = ahead_map.point.y;
        align_tf.transform.translation.z = 0.0;
        align_tf.transform.rotation = orientation;

        // CENTER frame
        geometry_msgs::msg::TransformStamped center_tf;
        center_tf.header.stamp = this->now();
        center_tf.header.frame_id = global_frame_;
        center_tf.child_frame_id = cart_center_frame_;
        center_tf.transform.translation.x = center_map.point.x;
        center_tf.transform.translation.y = center_map.point.y;
        center_tf.transform.translation.z = 0.0;
        center_tf.transform.rotation = orientation;

        static_transforms = {entry_tf, align_tf, center_tf};

        RCLCPP_INFO(this->get_logger(),
                    "Created frames from transformed laser coordinates");

      } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to transform coordinates: %s",
                     ex.what());
        return false;
      }
    }

    // Broadcast static transforms
    static_tf_broadcaster_->sendTransform(static_transforms);

    // throw std::runtime_error("~~~~~~~~~~~~~~~~~~~~DEGUGGING STOPPING cart "
    //                          "frame broadcasted~~~~~~~~~~~~~~~~~~~~");
    // Log frame positions
    for (const auto &tf : static_transforms) {
      RCLCPP_INFO(this->get_logger(), "  %s @ (%.2f, %.2f) in %s",
                  tf.child_frame_id.c_str(), tf.transform.translation.x,
                  tf.transform.translation.y, tf.header.frame_id.c_str());
    }

    // Wait for static transforms to propagate
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    return true;
  }

  // ============================================================================

  //   bool detect_and_create_cart_frames() {

  //     // Find shelf legs
  //     const std::vector<float> &intensity_data = latest_scan_->intensities;
  //     const std::vector<float> &ranges = latest_scan_->ranges;

  //     if (intensity_data.empty()) {
  //       RCLCPP_WARN(this->get_logger(), "No intensity data");
  //       return false;
  //     }

  //     // Calculate threshold
  //     float max_intensity =
  //         *std::max_element(intensity_data.begin(), intensity_data.end());
  //     float threshold = max_intensity * intensity_thres_fact_;

  //     RCLCPP_INFO(this->get_logger(), "max intensity found to be : %f",
  //                 max_intensity);
  //     RCLCPP_INFO(this->get_logger(), "Intensity threshold: %.2f",
  //     threshold);

  //     // Find legs
  //     struct ShelfLeg {
  //       size_t start, end;
  //     };
  //     std::vector<ShelfLeg> legs;
  //     auto iter = intensity_data.begin();

  //     while (iter != intensity_data.end() && legs.size() < 2) {
  //       iter = std::find_if(iter, intensity_data.end(),
  //                           [threshold](float v) { return v >= threshold; });
  //       if (iter == intensity_data.end())
  //         break;

  //       size_t start = std::distance(intensity_data.begin(), iter);

  //       iter = std::find_if(iter, intensity_data.end(),
  //                           [threshold](float v) { return v < threshold; });
  //       size_t end = (iter == intensity_data.end())
  //                        ? intensity_data.size() - 1
  //                        : std::distance(intensity_data.begin(), iter) - 1;

  //       legs.push_back({start, end});
  //       RCLCPP_INFO(this->get_logger(), "Leg %zu: [%zu, %zu]", legs.size(),
  //       start,
  //                   end);
  //     }

  //     if (legs.size() != 2) {
  //       RCLCPP_WARN(this->get_logger(), "Found %zu legs, need 2",
  //       legs.size()); return false;
  //     }

  //     // Calculate leg positions in laser frame
  //     size_t left_idx = (legs[0].start + legs[0].end) / 2;
  //     size_t right_idx = (legs[1].start + legs[1].end) / 2;

  //     float angle_left =
  //         latest_scan_->angle_min + left_idx * latest_scan_->angle_increment;
  //     float angle_right =
  //         latest_scan_->angle_min + right_idx *
  //         latest_scan_->angle_increment;

  //     float x1 = ranges[left_idx] * std::cos(angle_left);
  //     float y1 = ranges[left_idx] * std::sin(angle_left);
  //     float x2 = ranges[right_idx] * std::cos(angle_right);
  //     float y2 = ranges[right_idx] * std::sin(angle_right);

  //     // Calculate entry point and perpendicular vector
  //     float entry_x = (x1 + x2) / 2.0f;
  //     float entry_y = (y1 + y2) / 2.0f;

  //     float dx = x2 - x1;
  //     float dy = y2 - y1;
  //     float length = std::sqrt(dx * dx + dy * dy);
  //     float perp_x = -dy / length;
  //     float perp_y = dx / length;

  //     // Calculate frame positions in laser frame
  //     float ahead_x = entry_x + align_ahead_dist_ * perp_x;
  //     float ahead_y = entry_y + align_ahead_dist_ * perp_y;
  //     float center_x = entry_x - align_cart_center_dist_ * perp_x;
  //     float center_y = entry_y - align_cart_center_dist_ * perp_y;

  //     if (debug_) {
  //       RCLCPP_INFO(this->get_logger(), "=== Geometry Analysis ===");
  //       RCLCPP_INFO(this->get_logger(), "Leg 1: (%.2f, %.2f)", x1, y1);
  //       RCLCPP_INFO(this->get_logger(), "Leg 2: (%.2f, %.2f)", x2, y2);
  //       RCLCPP_INFO(this->get_logger(), "Entry: (%.2f, %.2f)", entry_x,
  //       entry_y); RCLCPP_INFO(this->get_logger(), "Perpendicular: (%.3f,
  //       %.3f)", perp_x,
  //                   perp_y);
  //     }

  //     // Try to use robot_cart_laser frame if available
  //     return create_static_cart_frames(entry_x, entry_y, ahead_x, ahead_y,
  //                                      center_x, center_y);
  //   }

  // ============================================================================
  // CREATE STATIC FRAMES
  // ============================================================================

  //   bool create_static_cart_frames(float entry_x, float entry_y, float
  //   ahead_x,
  //                                  float ahead_y, float center_x,
  //                                  float center_y) {

  //     const std::string ref_frame = "robot_cart_laser";
  //     bool use_reference = false;
  //     geometry_msgs::msg::TransformStamped ref_tf;

  //     // Check if robot_cart_laser is available
  //     if (!cart_ref_off_) {
  //       try {
  //         RCLCPP_INFO(this->get_logger(),
  //                     "Trying to fetch ref frame 0000000000000000");
  //         ref_tf = tf_buffer_->lookupTransform(
  //             global_frame_, // map
  //             ref_frame,     // robot_cart_laser (parent: odom)
  //             tf2::TimePointZero, tf2::durationFromSec(1.0));

  //         use_reference = true;
  //         RCLCPP_INFO(this->get_logger(),
  //                     "✅ Also, found %s frame, cloning position &
  //                     orientation", ref_frame.c_str());

  //       } catch (const tf2::TransformException &ex) {
  //         RCLCPP_WARN(this->get_logger(), "⚠️  %s not available: %s",
  //                     ref_frame.c_str(), ex.what());
  //         RCLCPP_INFO(this->get_logger(),
  //                     "Using fallback: transforming laser coordinates to %s",
  //                     global_frame_.c_str());
  //       }
  //     } else {
  //       RCLCPP_INFO(this->get_logger(),
  //                   "FORCING cart frame creating to use legs");
  //     }
  //     std::vector<geometry_msgs::msg::TransformStamped> static_transforms;

  //     if (use_reference) {
  //       //
  //       ========================================================================
  //       // METHOD 1: Clone robot_cart_laser frame with offsets
  //       //
  //       ========================================================================

  //       tf2::Transform T_ref;
  //       tf2::fromMsg(ref_tf.transform, T_ref);

  //       // ENTRY frame: Clone reference frame exactly
  //       geometry_msgs::msg::TransformStamped entry_tf;
  //       entry_tf.header.stamp = this->now();
  //       entry_tf.header.frame_id = global_frame_; // map
  //       entry_tf.child_frame_id = cart_entry_frame_;
  //       entry_tf.transform = tf2::toMsg(T_ref);

  //       // ALIGN frame: Offset forward (in reference frame's X axis)
  //       //   tf2::Transform T_align =
  //       //       T_ref * tf2::Transform(tf2::Quaternion::getIdentity(),
  //       //                              tf2::Vector3(align_ahead_dist_, 0.0,
  //       //                              0.0));
  //       tf2::Transform T_align =
  //           T_ref * tf2::Transform(tf2::Quaternion::getIdentity(),
  //                                  tf2::Vector3(-align_ahead_dist_, 0.0,
  //                                  0.0));

  //       geometry_msgs::msg::TransformStamped align_tf;
  //       align_tf.header.stamp = this->now();
  //       align_tf.header.frame_id = global_frame_; // map
  //       align_tf.child_frame_id = cart_ahead_frame_;
  //       align_tf.transform = tf2::toMsg(T_align);

  //       // CENTER frame: Offset backward (in reference frame's X axis)
  //       //   tf2::Transform T_center =
  //       //       T_ref *
  //       //       tf2::Transform(tf2::Quaternion::getIdentity(),
  //       //                      tf2::Vector3(-align_cart_center_dist_, 0.0,
  //       0.0)); tf2::Transform T_center =
  //           T_ref *
  //           tf2::Transform(tf2::Quaternion::getIdentity(),
  //                          tf2::Vector3(align_cart_center_dist_, 0.0, 0.0));

  //       geometry_msgs::msg::TransformStamped center_tf;
  //       center_tf.header.stamp = this->now();
  //       center_tf.header.frame_id = global_frame_; // map
  //       center_tf.child_frame_id = cart_center_frame_;
  //       center_tf.transform = tf2::toMsg(T_center);

  //       static_transforms = {entry_tf, align_tf, center_tf};

  //       RCLCPP_INFO(this->get_logger(), "Created frames by cloning %s",
  //                   ref_frame.c_str());

  //     } else {
  //       //
  //       ========================================================================
  //       // METHOD 2: Transform laser coordinates to map frame
  //       //
  //       ========================================================================

  //       std::string laser_frame = latest_scan_->header.frame_id;

  //       try {
  //         // Transform entry point
  //         geometry_msgs::msg::PointStamped entry_pt;
  //         entry_pt.header.frame_id = laser_frame;
  //         entry_pt.header.stamp = latest_scan_->header.stamp;
  //         entry_pt.point.x = entry_x;
  //         entry_pt.point.y = entry_y;
  //         entry_pt.point.z = 0.0;

  //         auto entry_map = tf_buffer_->transform(entry_pt, global_frame_,
  //                                                tf2::durationFromSec(1.0));

  //         // Transform ahead point
  //         geometry_msgs::msg::PointStamped ahead_pt;
  //         ahead_pt.header.frame_id = laser_frame;
  //         ahead_pt.header.stamp = latest_scan_->header.stamp;
  //         ahead_pt.point.x = ahead_x;
  //         ahead_pt.point.y = ahead_y;
  //         ahead_pt.point.z = 0.0;

  //         auto ahead_map = tf_buffer_->transform(ahead_pt, global_frame_,
  //                                                tf2::durationFromSec(1.0));

  //         // Transform center point
  //         geometry_msgs::msg::PointStamped center_pt;
  //         center_pt.header.frame_id = laser_frame;
  //         center_pt.header.stamp = latest_scan_->header.stamp;
  //         center_pt.point.x = center_x;
  //         center_pt.point.y = center_y;
  //         center_pt.point.z = 0.0;

  //         auto center_map = tf_buffer_->transform(center_pt, global_frame_,
  //                                                 tf2::durationFromSec(1.0));

  //         // Create orientation (facing into shelf, -90 degrees)
  //         tf2::Quaternion q;
  //         q.setRPY(0, 0, -M_PI / 2.0);
  //         geometry_msgs::msg::Quaternion orientation = tf2::toMsg(q);

  //         // ENTRY frame
  //         geometry_msgs::msg::TransformStamped entry_tf;
  //         entry_tf.header.stamp = this->now();
  //         entry_tf.header.frame_id = global_frame_;
  //         entry_tf.child_frame_id = cart_entry_frame_;
  //         entry_tf.transform.translation.x = entry_map.point.x;
  //         entry_tf.transform.translation.y = entry_map.point.y;
  //         entry_tf.transform.translation.z = 0.0;
  //         entry_tf.transform.rotation = orientation;

  //         // ALIGN frame
  //         geometry_msgs::msg::TransformStamped align_tf;
  //         align_tf.header.stamp = this->now();
  //         align_tf.header.frame_id = global_frame_;
  //         align_tf.child_frame_id = cart_ahead_frame_;
  //         align_tf.transform.translation.x = ahead_map.point.x;
  //         align_tf.transform.translation.y = ahead_map.point.y;
  //         align_tf.transform.translation.z = 0.0;
  //         align_tf.transform.rotation = orientation;

  //         // CENTER frame
  //         geometry_msgs::msg::TransformStamped center_tf;
  //         center_tf.header.stamp = this->now();
  //         center_tf.header.frame_id = global_frame_;
  //         center_tf.child_frame_id = cart_center_frame_;
  //         center_tf.transform.translation.x = center_map.point.x;
  //         center_tf.transform.translation.y = center_map.point.y;
  //         center_tf.transform.translation.z = 0.0;
  //         center_tf.transform.rotation = orientation;

  //         static_transforms = {entry_tf, align_tf, center_tf};

  //         RCLCPP_INFO(this->get_logger(),
  //                     "Created frames from transformed laser coordinates");

  //       } catch (const tf2::TransformException &ex) {
  //         RCLCPP_ERROR(this->get_logger(), "Failed to transform coordinates:
  //         %s",
  //                      ex.what());
  //         return false;
  //       }
  //     }

  //     // Broadcast static transforms
  //     static_tf_broadcaster_->sendTransform(static_transforms);

  //     // throw std::runtime_error("~~~~~~~~~~~~~~~~~~~~DEGUGGING STOPPING
  //     cart "
  //     //                          "frame broadcasted~~~~~~~~~~~~~~~~~~~~");
  //     // Log frame positions
  //     for (const auto &tf : static_transforms) {
  //       RCLCPP_INFO(this->get_logger(), "  %s @ (%.2f, %.2f) in %s",
  //                   tf.child_frame_id.c_str(), tf.transform.translation.x,
  //                   tf.transform.translation.y, tf.header.frame_id.c_str());
  //     }

  //     // Wait for static transforms to propagate
  //     rclcpp::sleep_for(std::chrono::milliseconds(200));

  //     return true;
  //   }

  void validate_tf_tree_consistency() {
    try {
      auto tf_map_odom = tf_buffer_->lookupTransform(global_frame_, odom_frame_,
                                                     tf2::TimePointZero,
                                                     tf2::durationFromSec(0.5));

      auto tf_cart_entry = tf_buffer_->lookupTransform(
          global_frame_, cart_entry_frame_, tf2::TimePointZero,
          tf2::durationFromSec(0.5));

      rclcpp::Time map_odom_time(tf_map_odom.header.stamp);
      rclcpp::Time cart_time(tf_cart_entry.header.stamp);

      RCLCPP_INFO(this->get_logger(), "=== TF Tree Validation ===");
      RCLCPP_INFO(this->get_logger(), "  map->odom:       %.3f",
                  map_odom_time.seconds());
      RCLCPP_INFO(this->get_logger(), "  map->cart_entry: %.3f",
                  cart_time.seconds());

      double diff = std::abs(map_odom_time.seconds() - cart_time.seconds());
      if (diff < 2.0) {
        RCLCPP_INFO(this->get_logger(), "✅ Timestamps consistent (diff: %.3fs)",
                    diff);
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "⚠️  Large timestamp difference: %.3fs", diff);
      }

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not validate TF: %s", ex.what());
    }
  }

  float get_laser_forward_min() {
    if (!latest_scan_ || latest_scan_->ranges.empty()) {
      return std::numeric_limits<float>::infinity();
    }

    const auto &scan = *latest_scan_;
    const float half_cone = front_side_band_ * M_PI / 180.0f;

    // Calculate window indices (same as before)
    const float zero_index_f = (0.0f - scan.angle_min) / scan.angle_increment;
    const int zero_index = static_cast<int>(std::round(zero_index_f));
    const int idx_offset =
        static_cast<int>(std::round(half_cone / scan.angle_increment));

    int start_idx = std::max(0, zero_index - idx_offset);
    int end_idx = std::min(static_cast<int>(scan.ranges.size()) - 1,
                           zero_index + idx_offset);

    if (start_idx > end_idx) {
      return std::numeric_limits<float>::infinity();
    }
    auto valid_min_it = std::min_element(
        scan.ranges.begin() + start_idx, scan.ranges.begin() + end_idx + 1,
        [scan = &scan](float a, float b) {
          // Skip invalid values (treat as "larger than any valid")
          auto is_valid = [&](float val) {
            return std::isfinite(val) && val >= scan->range_min &&
                   val <= scan->range_max;
          };

          if (is_valid(a) && !is_valid(b))
            return true; // a < b
          if (!is_valid(a) && is_valid(b))
            return false; // b < a
          if (!is_valid(a) && !is_valid(b))
            return false; // doesn't matter
          return a < b;   // both valid, normal comparison
        });

    float min_distance =
        std::isfinite(*valid_min_it) ? *valid_min_it : INFINITY;
    // RCLCPP_INFO(get_logger(), "Window [%d,%d]: min=%.3fm", start_idx,
    // end_idx,
    //             min_distance);

    return min_distance;
  }

  // NAVIGATIONS
  bool perform_final_approach() {
    RCLCPP_INFO(get_logger(), "=== Starting Final Approach ===");

    // Step 1: Align position
    RCLCPP_INFO(get_logger(), "Step 1/3: Moving to align position");
    if (!move_to_tf_frame(cart_ahead_frame_)) {
      RCLCPP_ERROR(get_logger(), "Failed to reach align position");
      stop_robot();
      return false;
    }
    RCLCPP_INFO(get_logger(), "✅✅✅ Reached Cart Align Frame");
    rclcpp::sleep_for(500ms);

    // Step 2: Entry position
    RCLCPP_INFO(get_logger(), "Step 2/3: Moving to entry position");
    if (!move_to_tf_frame(cart_entry_frame_)) {
      RCLCPP_ERROR(get_logger(), "Failed to reach entry");
      stop_robot();
      return false;
    }
    RCLCPP_INFO(get_logger(), "✅✅✅ Reached Cart Entry Frame");
    rclcpp::sleep_for(500ms);

    // Step 3: Center position
    RCLCPP_INFO(get_logger(), "Step 3/3: Moving to center");
    if (!move_to_tf_frame(cart_center_frame_)) {
      RCLCPP_ERROR(get_logger(), "Failed to reach center");
      stop_robot();
      return false;
    }
    RCLCPP_INFO(get_logger(), "✅✅✅ Reached Cart Center Frame");

    // Attach shelf
    // throw std::runtime_error("~~~~~~~~~~~~~~~~~~~~DEGUGGING STOPPING
    // attaching "
    //                          "shelf~~~~~~~~~~~~~~~~~~~~");
    bool attached = attach_shelf();

    RCLCPP_INFO(get_logger(), "Final Approach Complete");
    return attached;
  }

  bool move_to_tf_frame(const std::string &frame_name) {
    // Get goal pose
    double goal_x, goal_y, goal_yaw;
    if (!get_pose_from_tf(frame_name, goal_x, goal_y, goal_yaw)) {
      return false;
    }

    RCLCPP_INFO(get_logger(), "Target: (%.2f, %.2f, %.0f°)", goal_x, goal_y,
                goal_yaw * 180.0 / M_PI);

    // Phase 1: Rotate to face goal
    if (!rotate_to_face_goal(goal_x, goal_y))
      return false;

    // Phase 2: Drive to position
    if (!drive_to_position(goal_x, goal_y))
      return false;

    // Phase 3: Rotate to final angle
    if (!rotate_to_angle(goal_yaw))
      return false;

    return true;
  }

  bool get_pose_from_tf(const std::string &frame, double &x, double &y,
                        double &yaw) {
    try {
      auto tf =
          tf_buffer_->lookupTransform(global_frame_, frame, tf2::TimePointZero,
                                      std::chrono::milliseconds(500));

      x = tf.transform.translation.x;
      y = tf.transform.translation.y;

      tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                        tf.transform.rotation.z, tf.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch;
      m.getRPY(roll, pitch, yaw);

      return true;

    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "TF lookup failed for '%s': %s", frame.c_str(),
                   ex.what());
      return false;
    }
  }

  bool get_robot_pose(double &x, double &y, double &yaw) {
    try {
      auto tf = tf_buffer_->lookupTransform(global_frame_, base_frame_,
                                            tf2::TimePointZero);

      x = tf.transform.translation.x;
      y = tf.transform.translation.y;

      tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                        tf.transform.rotation.z, tf.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch;
      m.getRPY(roll, pitch, yaw);

      return true;
    } catch (const tf2::TransformException &ex) {
      return false;
    }
  }

  bool rotate_to_face_goal(double goal_x, double goal_y) {
    if (debug_restrict_move_) {
      RCLCPP_INFO(this->get_logger(), "Movement restricted (debug)");
      return true;
    }

    rclcpp::Rate rate(20);
    int timeout = 0;

    while (rclcpp::ok() && timeout++ < 200) {
      double x, y, yaw;
      if (!get_robot_pose(x, y, yaw)) {
        rate.sleep();
        continue;
      }

      double desired_yaw = atan2(goal_y - y, goal_x - x);
      double error = normalize_angle(desired_yaw - yaw);

      if (fabs(error) < angle_tolerance_) {
        stop_robot();
        RCLCPP_INFO(this->get_logger(), "✅✅ rotated for current goal");
        return true;
      }

      //   RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 500,
      //                        "\t- Phase 1: Rotating towards phase goal (%f >
      //                        %f)", error, angle_tolerance_);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 300,
                           "\t🚧 Phase 1: Rotating towards phase goal (%f > %f)",
                           error, angle_tolerance_);
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = copysign(angular_speed_, error);
      cmd_vel_publisher_->publish(cmd);

      rate.sleep();
    }

    return false;
  }

  bool drive_to_position(double goal_x, double goal_y) {
    if (debug_restrict_move_) {
      RCLCPP_INFO(this->get_logger(), "Movement restricted (debug)");
      return true;
    }

    rclcpp::Rate rate(20);
    int timeout = 0;

    while (rclcpp::ok() && timeout++ < 400) {
      double x, y, yaw;
      if (!get_robot_pose(x, y, yaw)) {
        rate.sleep();
        continue;
      }

      double dx = goal_x - x;
      double dy = goal_y - y;
      double distance = sqrt(dx * dx + dy * dy);

      if (distance < position_tolerance_) {
        stop_robot();
        RCLCPP_INFO(this->get_logger(), "✅✅ Reached position");
        return true;
      }

      double desired_yaw = atan2(dy, dx);
      double angle_error = normalize_angle(desired_yaw - yaw);

      RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 300,
                           "🚧 Phase2: Dist: %.2fm - PosTol: %.2f | Pos: "
                           "(%.2f,%.2f) → Goal: (%.2f,%.2f)",
                           distance, position_tolerance_, x, y, goal_x, goal_y);

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = linear_speed_;
      cmd.angular.z = std::max(-angular_speed_,
                               std::min(angular_speed_, 2.0 * angle_error));
      cmd_vel_publisher_->publish(cmd);

      rate.sleep();
    }

    return false;
  }

  bool rotate_to_angle(double goal_yaw) {
    if (debug_restrict_move_) {
      RCLCPP_INFO(this->get_logger(), "Movement restricted (debug)");
      return true;
    }

    rclcpp::Rate rate(20);
    int timeout = 0;

    while (rclcpp::ok() && timeout++ < 100) {
      double x, y, yaw;
      if (!get_robot_pose(x, y, yaw)) {
        rate.sleep();
        RCLCPP_INFO(this->get_logger(), "✅✅ aligned to next goal");
        continue;
      }

      double error = normalize_angle(goal_yaw - yaw);

      if (fabs(error) < angle_tolerance_) {
        stop_robot();
        return true;
      }

      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *get_clock(), 300,
          "\t🚧 Phase 3: Rotating towards next phase goal (%f > %f)", error,
          angle_tolerance_);
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = copysign(angular_speed_, error);
      cmd_vel_publisher_->publish(cmd);

      rate.sleep();
    }

    return false;
  }

  void stop_robot() {
    geometry_msgs::msg::Twist cmd;
    cmd_vel_publisher_->publish(cmd);
  }

  double normalize_angle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  bool move_forward_small_distance(float distance_meters = 0.05) {
    RCLCPP_INFO(get_logger(), "Moving forward %.3fm to retry leg detection...",
                distance_meters);

    if (debug_restrict_move_) {
      RCLCPP_INFO(get_logger(), "Movement restricted by debug flag");
      return true;
    }

    // Get initial position
    double start_x, start_y, start_yaw;
    if (!get_robot_pose(start_x, start_y, start_yaw)) {
      RCLCPP_ERROR(get_logger(), "Failed to get starting position");
      return false;
    }

    rclcpp::Rate rate(20); // 20 Hz
    int timeout = 0;
    const int max_timeout = 100; // 5 seconds max

    while (rclcpp::ok() && timeout++ < max_timeout) {

      // Get current position
      double current_x, current_y, current_yaw;
      if (!get_robot_pose(current_x, current_y, current_yaw)) {
        rate.sleep();
        continue;
      }

      // Calculate distance traveled
      double dx = current_x - start_x;
      double dy = current_y - start_y;
      double distance_traveled = sqrt(dx * dx + dy * dy);

      // Check if reached target distance
      if (distance_traveled >= distance_meters) {
        stop_robot();
        RCLCPP_INFO(get_logger(), "Moved forward %.3fm", distance_traveled);

        // Small delay to let robot settle
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        return true;
      }

      // Drive forward slowly
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.05;
      cmd.angular.z = 0.0;
      cmd_vel_publisher_->publish(cmd);

      rate.sleep();
    }

    // Timeout
    stop_robot();
    RCLCPP_WARN(get_logger(), "Timeout while moving forward");
    return false;
  }

  bool attach_shelf() {
    if (debug_restrict_attach_shelf_) {
      RCLCPP_INFO(this->get_logger(), "Shelf attach restricted (debug)");
      return true;
    }

    rclcpp::sleep_for(500ms);

    auto elevator_pub =
        this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);
    rclcpp::sleep_for(1000ms);

    std_msgs::msg::String msg;
    msg.data = "";

    for (int i = 0; i < 3; i++) {
      elevator_pub->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Elevator command %d/3", i + 1);
      rclcpp::sleep_for(500ms);
    }

    rclcpp::sleep_for(5500ms);
    RCLCPP_INFO(this->get_logger(), "✅ Shelf attached");

    return true;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachServiceServer>());
  rclcpp::shutdown();
  return 0;
}
