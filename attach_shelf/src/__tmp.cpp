// Sequential TF Navigation WITHOUT Nav2 - Using Direct Velocity Control

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class ApproachServiceServer : public rclcpp::Node
{
public:
    ApproachServiceServer() : Node("approach_service_server")
    {
        // TF setup
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        align_static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        cart_static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        // Velocity command publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        // Odometry subscriber to get current robot pose
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&ApproachServiceServer::odom_callback, this, std::placeholders::_1));
        
        // Control parameters (tune these for your robot)
        linear_speed_ = 0.2;      // m/s - slower for tight spaces
        angular_speed_ = 0.3;     // rad/s
        position_tolerance_ = 0.05;  // 5cm tolerance
        angle_tolerance_ = 0.1;      // ~5.7 degrees
        
        RCLCPP_INFO(get_logger(), "Approach Service Server initialized (No Nav2)");
    }
    
    void detect_and_broadcast_cart_frame(double cart_x, double cart_y, double cart_yaw)
    {
        RCLCPP_INFO(get_logger(), "Broadcasting cart frames");
        
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        
        // Create align position (0.5m before cart)
        geometry_msgs::msg::TransformStamped align_tf;
        align_tf.header.stamp = this->now();
        align_tf.header.frame_id = "map";
        align_tf.child_frame_id = "align_static_tf";
        
        double offset = 0.5;
        align_tf.transform.translation.x = cart_x - offset * cos(cart_yaw);
        align_tf.transform.translation.y = cart_y - offset * sin(cart_yaw);
        align_tf.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, cart_yaw);
        align_tf.transform.rotation = tf2::toMsg(q);
        
        transforms.push_back(align_tf);
        
        // Create cart position
        geometry_msgs::msg::TransformStamped cart_tf;
        cart_tf.header.stamp = this->now();
        cart_tf.header.frame_id = "map";
        cart_tf.child_frame_id = "cart_static_tf";
        
        cart_tf.transform.translation.x = cart_x;
        cart_tf.transform.translation.y = cart_y;
        cart_tf.transform.translation.z = 0.0;
        cart_tf.transform.rotation = tf2::toMsg(q);
        
        transforms.push_back(cart_tf);
        
        // Broadcast both transforms
        align_static_tf_broadcaster_->sendTransform(transforms[0]);
        cart_static_tf_broadcaster_->sendTransform(transforms[1]);
        
        RCLCPP_INFO(get_logger(), 
            "Broadcasted align at (%.2f, %.2f) and cart at (%.2f, %.2f)",
            align_tf.transform.translation.x,
            align_tf.transform.translation.y,
            cart_tf.transform.translation.x,
            cart_tf.transform.translation.y);
        
        // Wait for TF buffer to update
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
    
    // =================================================================
    // MAIN METHOD: Sequential navigation to both TF frames
    // =================================================================
    void perform_final_approach()
    {
        RCLCPP_INFO(get_logger(), "=== Starting Final Approach (Direct Control) ===");
        
        // STEP 1: Navigate to align position
        RCLCPP_INFO(get_logger(), "Step 1/2: Moving to align position...");
        
        if (!move_to_tf_frame("align_static_tf")) {
            RCLCPP_ERROR(get_logger(), "❌ Failed to reach align position");
            stop_robot();
            return;
        }
        
        RCLCPP_INFO(get_logger(), "✓ Step 1 Complete: Reached align position!");
        
        // Small pause between waypoints
        stop_robot();
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        
        // STEP 2: Navigate to cart position
        RCLCPP_INFO(get_logger(), "Step 2/2: Moving to cart position...");
        
        if (!move_to_tf_frame("cart_static_tf")) {
            RCLCPP_ERROR(get_logger(), "❌ Failed to reach cart position");
            stop_robot();
            return;
        }
        
        RCLCPP_INFO(get_logger(), "✓ Step 2 Complete: Reached cart position!");
        stop_robot();
        
        RCLCPP_INFO(get_logger(), "=== Final Approach Complete! ===");
    }

private:
    // Publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // TF members
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> align_static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> cart_static_tf_broadcaster_;
    
    // Current robot pose (from odometry)
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    bool odom_received_ = false;
    
    // Control parameters
    double linear_speed_;
    double angular_speed_;
    double position_tolerance_;
    double angle_tolerance_;
    
    // =================================================================
    // Odometry callback to track robot position
    // =================================================================
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
        
        odom_received_ = true;
    }
    
    // =================================================================
    // Move to a TF frame using direct velocity control
    // =================================================================
    bool move_to_tf_frame(const std::string& frame_name)
    {
        // Wait for odometry
        if (!wait_for_odometry()) {
            RCLCPP_ERROR(get_logger(), "No odometry received!");
            return false;
        }
        
        // Get goal pose from TF
        double goal_x, goal_y, goal_yaw;
        if (!get_pose_from_tf(frame_name, goal_x, goal_y, goal_yaw)) {
            return false;
        }
        
        RCLCPP_INFO(get_logger(), 
            "Target: (%.2f, %.2f, %.2f°) | Current: (%.2f, %.2f, %.2f°)",
            goal_x, goal_y, goal_yaw * 180.0 / M_PI,
            current_x_, current_y_, current_yaw_ * 180.0 / M_PI);
        
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
    
    // =================================================================
    // Phase 1: Rotate to face the goal position
    // =================================================================
    bool rotate_to_face_goal(double goal_x, double goal_y)
    {
        RCLCPP_INFO(get_logger(), "Phase 1: Rotating to face goal...");
        
        rclcpp::Rate rate(20);  // 20 Hz control loop
        int timeout_count = 0;
        const int max_timeout = 100;  // 5 seconds at 20Hz
        
        while (rclcpp::ok()) {
            // Calculate desired angle to goal
            double dx = goal_x - current_x_;
            double dy = goal_y - current_y_;
            double desired_yaw = atan2(dy, dx);
            
            // Calculate angle error
            double angle_error = normalize_angle(desired_yaw - current_yaw_);
            
            // Check if we're facing the goal
            if (fabs(angle_error) < angle_tolerance_) {
                stop_robot();
                RCLCPP_INFO(get_logger(), "✓ Facing goal");
                return true;
            }
            
            // Send rotation command
            geometry_msgs::msg::Twist cmd;
            cmd.angular.z = copysign(angular_speed_, angle_error);
            cmd_vel_pub_->publish(cmd);
            
            // Safety timeout
            if (++timeout_count > max_timeout) {
                RCLCPP_WARN(get_logger(), "Rotation timeout");
                stop_robot();
                return false;
            }
            
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
        
        return false;
    }
    
    // =================================================================
    // Phase 2: Drive forward to the goal position
    // =================================================================
    bool drive_to_position(double goal_x, double goal_y)
    {
        RCLCPP_INFO(get_logger(), "Phase 2: Driving to position...");
        
        rclcpp::Rate rate(20);  // 20 Hz control loop
        int timeout_count = 0;
        const int max_timeout = 200;  // 10 seconds at 20Hz
        
        while (rclcpp::ok()) {
            // Calculate distance to goal
            double dx = goal_x - current_x_;
            double dy = goal_y - current_y_;
            double distance = sqrt(dx * dx + dy * dy);
            
            // Check if we've reached the position
            if (distance < position_tolerance_) {
                stop_robot();
                RCLCPP_INFO(get_logger(), "✓ Reached position");
                return true;
            }
            
            // Calculate heading error (to maintain direction)
            double desired_yaw = atan2(dy, dx);
            double angle_error = normalize_angle(desired_yaw - current_yaw_);
            
            // Send drive command with heading correction
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = linear_speed_;
            cmd.angular.z = 2.0 * angle_error;  // Proportional heading correction
            
            // Limit angular velocity
            cmd.angular.z = std::max(-angular_speed_, 
                                     std::min(angular_speed_, cmd.angular.z));
            
            cmd_vel_pub_->publish(cmd);
            
            // Safety timeout
            if (++timeout_count > max_timeout) {
                RCLCPP_WARN(get_logger(), "Drive timeout");
                stop_robot();
                return false;
            }
            
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
        
        return false;
    }
    
    // =================================================================
    // Phase 3: Rotate to final orientation
    // =================================================================
    bool rotate_to_angle(double goal_yaw)
    {
        RCLCPP_INFO(get_logger(), "Phase 3: Rotating to final orientation...");
        
        rclcpp::Rate rate(20);  // 20 Hz control loop
        int timeout_count = 0;
        const int max_timeout = 100;  // 5 seconds at 20Hz
        
        while (rclcpp::ok()) {
            // Calculate angle error
            double angle_error = normalize_angle(goal_yaw - current_yaw_);
            
            // Check if we're at the correct angle
            if (fabs(angle_error) < angle_tolerance_) {
                stop_robot();
                RCLCPP_INFO(get_logger(), "✓ Final orientation achieved");
                return true;
            }
            
            // Send rotation command
            geometry_msgs::msg::Twist cmd;
            cmd.angular.z = copysign(angular_speed_, angle_error);
            cmd_vel_pub_->publish(cmd);
            
            // Safety timeout
            if (++timeout_count > max_timeout) {
                RCLCPP_WARN(get_logger(), "Final rotation timeout");
                stop_robot();
                return false;
            }
            
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
        
        return false;
    }
    
    // =================================================================
    // Helper functions
    // =================================================================
    
    bool get_pose_from_tf(const std::string& target_frame,
                          double& x, double& y, double& yaw)
    {
        try {
            auto transform = tf_buffer_->lookupTransform(
                "map",
                target_frame,
                tf2::TimePointZero,
                rclcpp::Duration::from_seconds(1.0)
            );
            
            x = transform.transform.translation.x;
            y = transform.transform.translation.y;
            
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );
            
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, yaw);
            
            return true;
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(get_logger(), 
                "Failed to get TF for '%s': %s", 
                target_frame.c_str(), ex.what());
            return false;
        }
    }
    
    bool wait_for_odometry()
    {
        rclcpp::Rate rate(10);
        int timeout = 50;  // 5 seconds
        
        while (!odom_received_ && timeout > 0 && rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
            timeout--;
        }
        
        return odom_received_;
    }
    
    void stop_robot()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
    }
    
    double normalize_angle(double angle)
    {
        // Normalize angle to [-pi, pi]
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ApproachServiceServer>();
    
    // Example usage:
    // 1. Detect cart and broadcast TFs
    node->detect_and_broadcast_cart_frame(2.0, 1.5, 0.0);
    
    // 2. Perform final approach
    node->perform_final_approach();
    
    rclcpp::shutdown();
    return 0;
}