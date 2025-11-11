import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

class WaypointCommander(Node):
    def __init__(self):
        super().__init__('waypoint_commander')

        self.navigator = BasicNavigator()

        # Publisher for elevator or any custom topic
        self.elevator_pub = self.create_publisher(String, '/elevator_down', 10)

        # Define waypoints
        self.waypoints = self.define_waypoints()

    def define_waypoints(self):
        # Replace with your own waypoints
        waypoints = []
        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.pose.position.x = 1.0
        pose1.pose.position.y = 0.0
        pose1.pose.orientation.w = 1.0

        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        pose2.pose.position.x = 3.0
        pose2.pose.position.y = 0.0
        pose2.pose.orientation.w = 1.0

        waypoints.append(pose1)
        waypoints.append(pose2)
        return waypoints

    def run(self):
        self.navigator.waitUntilNav2Active()

        print("Starting waypoint navigation...")
        self.navigator.goThroughPoses(self.waypoints)

        current_wp = 0

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                # Detect if we’re close to a specific waypoint
                dist = feedback.distance_remaining
                if dist < 0.5 and current_wp == 0:
                    self.trigger_elevator()
                    current_wp += 1

            time.sleep(0.5)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Route completed successfully!")

    def trigger_elevator(self):
        print("🔽 Triggering elevator down command...")
        msg = String()
        msg.data = "down"
        self.elevator_pub.publish(msg)
        time.sleep(0.5)  # give it a moment to publish
        print("Elevator command sent ✅")

def main():
    rclpy.init()
    node = WaypointCommander()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
