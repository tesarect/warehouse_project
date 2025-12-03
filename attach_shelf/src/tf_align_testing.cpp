#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class StaticGoalFollower : public rclcpp::Node
{
public:
  StaticGoalFollower()
  : Node("static_goal_follower")
  {
    // 1) Create static broadcaster and publish static goal frame
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    geometry_msgs::msg::TransformStamped goal_tf;
    goal_tf.header.stamp = this->get_clock()->now();
    goal_tf.header.frame_id = "map";        // parent
    goal_tf.child_frame_id = "goal_frame";  // static goal

    // Example goal pose in map frame
    goal_tf.transform.translation.x = 2.0;
    goal_tf.transform.translation.y = 1.0;
    goal_tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, M_PI / 2.0);  // 90 deg yaw
    goal_tf.transform.rotation.x = q.x();
    goal_tf.transform.rotation.y = q.y();
    goal_tf.transform.rotation.z = q.z();
    goal_tf.transform.rotation.w = q.w();

    static_broadcaster_->sendTransform(goal_tf);

    RCLCPP_INFO(get_logger(), "Published static goal_frame in map");

    // 2) Setup tf2 buffer + listener to query transforms
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 3) Periodic timer to check robot vs goal_frame
    timer_ = this->create_wall_timer(
      200ms, std::bind(&StaticGoalFollower::timerCallback, this));
  }

private:
  void timerCallback()
  {
    geometry_msgs::msg::TransformStamped base_in_map;
    geometry_msgs::msg::TransformStamped goal_in_map;

    try {
      // latest available transforms
      base_in_map = tf_buffer_->lookupTransform(
        "map", "base_link", tf2::TimePointZero);
      goal_in_map = tf_buffer_->lookupTransform(
        "map", "goal_frame", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF lookup failed: %s", ex.what());
      return;
    }

    // 1) Position error in map frame
    double dx = goal_in_map.transform.translation.x - base_in_map.transform.translation.x;
    double dy = goal_in_map.transform.translation.y - base_in_map.transform.translation.y;
    double pos_error = std::hypot(dx, dy);

    // 2) Yaw error robot vs goal
    double base_yaw = getYaw(base_in_map.transform.rotation);
    double goal_yaw = getYaw(goal_in_map.transform.rotation);
    double yaw_error = normalizeAngle(goal_yaw - base_yaw);

    // Thresholds for "reached" and "aligned"
    constexpr double POS_THRESH = 0.05;    // 5 cm
    constexpr double YAW_THRESH = 5.0 * M_PI / 180.0;  // 5 degrees

    bool at_position = pos_error < POS_THRESH;
    bool aligned = std::fabs(yaw_error) < YAW_THRESH;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "pos_error=%.3f m, yaw_error=%.2f deg, at_pos=%d, aligned=%d",
      pos_error,
      yaw_error * 180.0 / M_PI,
      at_position, aligned);

    // Here you would add your motion command logic, e.g.:
    // - If not at_position: drive towards goal position
    // - Else if not aligned: rotate in place
    // - Else: stop and report goal reached
  }

  static double getYaw(const geometry_msgs::msg::Quaternion & q_msg)
  {
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  static double normalizeAngle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticGoalFollower>());
  rclcpp::shutdown();
  return 0;
}
