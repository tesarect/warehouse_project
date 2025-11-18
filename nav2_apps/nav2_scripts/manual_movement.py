import math
import time
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class ManualMover(Node):
    """A simple robot movement helper class for ROS2."""

    def __init__(self, cmd_vel_topic):
        super().__init__('manual_mover')

        self.cmd_vel_topic = cmd_vel_topic
        
        # Get list of all topics
        # self._topic_list = self.get_topic_names_and_types()
        # print(f'--- topic list : {self._topic_list}')

        # if '/cmd_vel' in [topic[0] for topic in self._topic_list]:
        #     self.cmd_vel_topic = '/cmd_vel'
        # self.scan_topics()

        # Load respective command velocity throug self/Node.get_parameter(x).vale
        print(f'-----{self.cmd_vel_topic}')

        # # Create publisher for cmd_vel
        # self.cmd_vel_pub = self.create_publisher(
        #     Twist,
        #     '/diffbot_base_controller/cmd_vel_unstamped',
        #     10
        # )        
        # Create publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

    def scan_topics(self):
        _try = 1
        while not self.cmd_vel_topic:
            self._topic_list = self.get_topic_names_and_types()
            if len(self._topic_list) > 7:
                if '/cmd_vel' in [topic[0] for topic in self._topic_list]:
                    self.cmd_vel_topic = '/cmd_vel'
                    # print(f' ----try {_try}')
                else:
                    self.cmd_vel_topic = '/diffbot_base_controller/cmd_vel_unstamped'
                    # print(f' ----try {_try}')
                break
            _try += 1


    def inplace_rotation(self, rotate_deg: float, rotate_speed: float = 0.5):
        """
        Rotate robot in place by given degrees (positive = left, negative = right).
        rotate_deg: degrees to rotate (float)
        rotate_speed: angular speed in rad/s
        """
        vel_msg = Twist()

        # Convert degrees to radians
        rotate_rad = math.radians(abs(rotate_deg))

        # Compute rotation duration (t = θ / ω)
        duration = rotate_rad / rotate_speed

        # Determine direction
        vel_msg.angular.z = rotate_speed if rotate_deg > 0 else -rotate_speed
        vel_msg.linear.x = 0.0

        self.get_logger().info(
            f"Rotating {rotate_deg} degrees for {duration:.2f} seconds..."
        )

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(vel_msg)
            time.sleep(0.1)

        self.stop()
        self.get_logger().info("Rotation complete")

    def move_backward(self, distance: float = 0.5, curvature_deg: float = 0.0, speed: float = 0.2):
        """
        Move the robot backward for a specific distance.
        distance: meters to move backward
        curvature_deg: desired curvature in DEGREES per second (positive = left curve, negative = right curve)
        speed: linear velocity (m/s)
        """

        # Convert curvature from degrees/s to radians/s
        curvature_rad = math.radians(curvature_deg)

        vel_msg = Twist()
        vel_msg.linear.x = -abs(speed)
        vel_msg.angular.z = curvature_rad

        duration = distance / speed

        self.get_logger().info(
            f"Moving backward {distance} m at {speed} m/s for {duration:.2f} seconds..."
        )

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(vel_msg)
            time.sleep(0.1)

        self.stop()
        self.get_logger().info("Backward motion complete ")

    def move_forward(self, distance: float = 0.5, curvature_deg: float = 0.0, speed: float = 0.2):
        """
        Move the robot backward for a specific distance.
        distance: meters to move backward
        curvature_deg: desired curvature in DEGREES per second (positive = left curve, negative = right curve)
        speed: linear velocity (m/s)
        """

        # Convert curvature from degrees/s to radians/s
        curvature_rad = math.radians(curvature_deg)

        vel_msg = Twist()
        vel_msg.linear.x = abs(speed)
        vel_msg.angular.z = curvature_rad

        duration = distance / speed

        self.get_logger().info(
            f"Moving backward {distance} m at {speed} m/s for {duration:.2f} seconds..."
        )

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(vel_msg)
            time.sleep(0.1)

        self.stop()
        self.get_logger().info("Backward motion complete ")


    def stop(self):
        """Stop the robot by sending multiple zero-velocity messages."""
        stop_msg = Twist()
        for _ in range(10):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.1)
        self.get_logger().info("Robot stopped")


# Example usage
def main(args=None):
    rclpy.init(args=args)
    mover = ManualMover()

    # Example commands
    # mover.move_forward(distance=1.5, speed=0.50)
    # mover.move_backward(distance=0.4, speed=0.25)
    # mover.inplace_rotation(rotate_deg=45, rotate_speed=0.5)
    # mover.inplace_rotation(rotate_deg=-30, rotate_speed=0.5)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
