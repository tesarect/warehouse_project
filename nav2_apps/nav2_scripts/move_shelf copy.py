import time
import os
import yaml
from inspect import currentframe as inspectCurFrame
from copy import deepcopy
from math import pi as PI

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.srv import ClearEntireCostmap
from std_msgs.msg import String

from tf_transformations import quaternion_from_euler
from rcl_interfaces.srv import SetParameters
from ament_index_python.packages import get_package_share_directory

from attach_shelf.srv import GoToLoading
from manual_movement import ManualMover

class ShelfApproachManager(Node):
    # _initial_position = [0.0306187, 0.0211561, 0.00883951]
    # _loading_position = [5.7098795, -0.0307365, -0.6950259, 0.7189846]
    # _shipping_position = [2.465, 1.500, 0.707, 0.706]
    # _throughPoint1_position = [2.365, 0.188, 0.711, 0.702]

    # _footprint_with_shelf = "[[0.41, 0.41], [0.41, -0.41], [-0.41, -0.41], [-0.41, 0.41]]"
    # _footprint_without_shelf = "[[0.26, 0.26], [0.26, -0.26], [-0.26, -0.26], [-0.26, 0.26]]"

    def __init__(self, robot='sim'):
        rclpy.init()
        super().__init__('shelf_approach_manager')

        self.robot = robot
        self.load_parameters()
        self.maneuver = ManualMover(self.cmd_vel_topic)
        self.navigator = BasicNavigator()
        # self.declare_parameter("cmd_vel_topic", "")

        # self.pkg_path = get_package_share_directory('robot_config')
        # self.param_file = os.path.join(self.pkg_path, 'config', 'sim.yaml')
        # print(f"Loading parameters from: {self.param_file}")
        # self.yaml_params = self.load_yaml_file(self.param_file)
        # self.params = self.yaml_params['/**']['ros__parameters']

        # self.odom_frame = self.get_parameter('odom_frame').value
        # self.base_frame = self.get_parameter('base_frame').value
        # self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # self.get_logger().info(f"Loaded odom_frame = {self.odom_frame}")
        # self.get_logger().info(f"Loaded base_frame = {self.base_frame}")
        # self.get_logger().info(f"Loaded cmd_vel_topic = {self.cmd_vel_topic}")

        self.approach_client = self.create_client(GoToLoading, 'approach_shelf')
        self.param_client = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')

        self.elevatorDown_pub = self.create_publisher(String, '/elevator_down', 10)

    def load_yaml_file(self, yaml_path):
        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)

    def load_parameters(self):
        _pkg_path = get_package_share_directory('robot_config')
        param_file = os.path.join(_pkg_path, f'config', f'{self.robot}.yaml')
        print(f"Loading parameters from: {param_file}")
        yaml_params = self.load_yaml_file(param_file)
        params = yaml_params['/**']['ros__parameters']
        
        for key, value in params.items():
            self.declare_parameter(key, value)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.get_logger().info(f"Loaded odom_frame = {self.odom_frame}")

        self.base_frame = self.get_parameter('base_frame').value
        self.get_logger().info(f"Loaded base_frame = {self.base_frame}")
        
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.get_logger().info(f"Loaded cmd_vel_topic = {self.cmd_vel_topic}")
        
        self._initial_position = self.get_parameter('initial_position').value
        self.get_logger().info(f"Loaded initial_position = {self._initial_position}")
        
        self._loading_position = self.get_parameter('loading_position').value
        self.get_logger().info(f"Loaded loading_position = {self._loading_position}")
        
        self._shipping_position = self.get_parameter('shipping_position').value
        self.get_logger().info(f"Loaded shipping_position = {self._shipping_position}")
        
        self._throughPoint1_position = self.get_parameter('throughPoint1_position').value
        self.get_logger().info(f"Loaded throughPoint1_position = {self._throughPoint1_position}")
        
        self._footprint_with_shelf = self.get_parameter('footprint_with_shelf').value
        self.get_logger().info(f"Loaded footprint_with_shelf = {self._footprint_with_shelf}")
        
        self._footprint_without_shelf = self.get_parameter('footprint_without_shelf').value
        self.get_logger().info(f"Loaded footprint_without_shelf = {self._footprint_without_shelf}")

    def update_footprint_for_shelf(self, shelf):
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            print('Parameter service not available, waiting...')
        if shelf:
            footprint = self._footprint_with_shelf
            footprint_clearing = False
        else:
            footprint = self._footprint_without_shelf
            footprint_clearing = True
        parameters = [
            Parameter(name='footprint', value=footprint),
            Parameter(name='footprint_clearing_enabled', value=footprint_clearing)
        ]
        ros_parameters = [Parameter.to_parameter_msg(param) for param in parameters]
        request = SetParameters.Request()
        request.parameters = ros_parameters
        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(0.5)
        if future.result():
            if shelf:
                print('Footprint updated WITH shelf')
            else:
                print('Footprint updated WITHOUT shelf')
        return future.result()

    def clear_costmaps(self):
        local_clear_client = self.create_client(ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap')
        while not local_clear_client.wait_for_service(timeout_sec=1.0):
            print('Clear local costmap service not available, waiting...')
        request = ClearEntireCostmap.Request()
        local_future = local_clear_client.call_async(request)
        rclpy.spin_until_future_complete(self, local_future)
        print("Cleared local costmap")

        global_clear_client = self.create_client(ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap')
        while not global_clear_client.wait_for_service(timeout_sec=1.0):
            print('Clear global costmap service not available, waiting...')
        global_future = global_clear_client.call_async(ClearEntireCostmap.Request())
        rclpy.spin_until_future_complete(self, global_future)
        print("Cleared global costmap")
        return True

    def bot_approach_attach_shelf(self):
        while not self.approach_client.wait_for_service(timeout_sec=1.0):
            print('Approach service not available, waiting...')
        attach_shelf_request = GoToLoading.Request(attach_to_shelf=True)
        print('`/approach_shelf` service calling initiated...')
        future = self.approach_client.call_async(attach_shelf_request)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            resp = future.result()
            if resp is not None and resp.complete:
                print('Move underneath shelf and attached SUCCESSFUL')
                self.update_footprint_for_shelf(shelf=True)
                self.clear_costmaps()
                return True
            else:
                print('Move underneath shelf and attached FAILURE')
                return False
        else:
            print('approach_shelf service call FAILURE')
            if future.exception():
                print(f'Exception: {future.exception()}')
            return False

    def goToLocation(self, position, action=None):
        
        _pose = self.create_pose(position)
        # _pose = PoseStamped()
        # _pose.header.frame_id = 'map'
        # _pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # _pose.pose.position.x = position[0]
        # _pose.pose.position.y = position[1]
        # try:
        #     _pose.pose.orientation.z = position[2]
        #     _pose.pose.orientation.w = position[3]
        # except:
        #     print(f'No orientation available for {_pose_name} ')
        self.navigator.goToPose(_pose)
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival : {0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.')
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f' << Reached and looking for any')
            if action:
                print('Received action to be executed after goal reached')
                self.trigger_elevator()
        elif result == TaskResult.CANCELED:
            print('Task was canceled. Returning to staging point...')
            return False
        elif result == TaskResult.FAILED:
            print('Task failed!')
            exit(-1)
        # print(f' << Reached {_pose_name}')
        return True

    def trigger_elevator(self):
        print("Triggering elevator down command...")
        msg = String()
        msg.data = "down"
        self.elevatorDown_pub.publish(msg)
        time.sleep(1.0)
        print("Elevator command sent")
    
    def create_pose(self, position):
        _pose = PoseStamped()
        _pose.header.frame_id = 'map'
        _pose.header.stamp = self.navigator.get_clock().now().to_msg()
        _pose.pose.position.x = position[0]
        _pose.pose.position.y = position[1]
        # q = quaternion_from_euler(0.0, 0.0, position[2])
        # _pose.pose.orientation.x = q[0]
        # _pose.pose.orientation.y = q[1]
        _pose.pose.orientation.z = position[2]
        _pose.pose.orientation.w = position[3]
        
        return _pose

    def set_initial_pose(self):
        self.initial_pose = self.create_pose(self._initial_position)
        # self.initial_pose = PoseStamped()
        # self.initial_pose.header.frame_id = 'map'
        # self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # self.initial_pose.pose.position.x = self._initial_position[0]
        # self.initial_pose.pose.position.y = self._initial_position[1]
        # q = quaternion_from_euler(0.0, 0.0, self._initial_position[2])
        # self.initial_pose.pose.orientation.x = q[0]
        # self.initial_pose.pose.orientation.y = q[1]
        # self.initial_pose.pose.orientation.z = q[2]
        # self.initial_pose.pose.orientation.w = q[3]
        self.navigator.setInitialPose(self.initial_pose)
        self.navigator.waitUntilNav2Active()

    def routine(self):

        # Set initial pose
        self.set_initial_pose()
        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = 'map'
        # initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # initial_pose.pose.position.x = self._initial_position[0]
        # initial_pose.pose.position.y = self._initial_position[1]
        # q = quaternion_from_euler(0.0, 0.0, self._initial_position[2])
        # initial_pose.pose.orientation.x = q[0]
        # initial_pose.pose.orientation.y = q[1]
        # initial_pose.pose.orientation.z = q[2]
        # initial_pose.pose.orientation.w = q[3]
        # self.navigator.setInitialPose(initial_pose)
        # self.navigator.waitUntilNav2Active()

        # Go to loading position
        load_pose = self.create_pose(self._loading_position)
        # load_pose = PoseStamped()
        # load_pose.header.frame_id = 'map'
        # load_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # load_pose.pose.position.x = self._loading_position[0]
        # load_pose.pose.position.y = self._loading_position[1]
        # load_pose.pose.orientation.z = self._loading_position[2]
        # load_pose.pose.orientation.w = self._loading_position[3]
        self.navigator.goToPose(load_pose)
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 10 == 0:
                print('Estimated time of arrival at loading position ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.')
        result = self.navigator.getResult()
        if result == TaskResult.CANCELED:
            print('Task was canceled. Returning to staging point...')
            self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.navigator.goToPose(self.initial_pose)
        elif result == TaskResult.FAILED:
            print('Task failed!')
            exit(-1)
        elif result == TaskResult.SUCCEEDED:
            print('Reached loading position')
            _shelf_attached = self.bot_approach_attach_shelf()

        while not self.navigator.isTaskComplete():
            time.sleep(0.5)
            print('Navigator GoToLoading COMPLETED')

        if _shelf_attached:
            self.maneuver.move_backward(distance=1.40, curvature_deg=6.0, speed=0.3)
            time.sleep(0.5)
            self.maneuver.inplace_rotation(rotate_deg=-180, rotate_speed=0.5)
            time.sleep(0.5)
            self.goToLocation(position=self._throughPoint1_position)
            self.goToLocation(position=self._shipping_position, action=True)
            self.update_footprint_for_shelf(shelf=False)
            self.maneuver.move_backward(distance=1.4, speed=0.3)
            time.sleep(0.5)
            self.clear_costmaps()
            self.maneuver.inplace_rotation(rotate_deg=90, rotate_speed=0.5)
            time.sleep(0.5)
        else:
            print('Attach shelf FAILURE. Returing to initial position')
        self.goToLocation(position=self._initial_position)
        while not self.navigator.isTaskComplete():
            pass
        exit(0)

if __name__ == '__main__':
    manager = ShelfApproachManager()
    manager.run()
