import time
import math
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler
import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from attach_shelf.srv import GoToLoading
from geometry_msgs.msg import Twist

from odom_listener import OdomListener

# Taken from amcl config
_initial_position = [0.0306187, 0.0211561, 0.00883951]

# Obtained from `/goal_pose` [x,y,z,w] (x & y are pose, z & w are orientation)
_loading_position = [5.7098795, -0.1307365, -0.6950259, 0.7189846]
_shipping_position = [2.5659244, 1.3705197, 0.7073400, 0.7068734]
_throughPoint1_position = [2.6281423, 0.1889415, 0.7113135, 0.7028748]


_footprint_with_shelf = "[[0.40, 0.40], [0.40, -0.40], [-0.40, -0.40], [-0.40, 0.40]]"
_footprint_without_shelf = "[[0.26, 0.26], [0.26, -0.26], [-0.26, -0.26], [-0.26, 0.26]]"


rclpy.init()
navigator = BasicNavigator()

# Create a node for service calls
shelf_attacing_node = rclpy.create_node('move_shelf')

# Create the service client - adjust the service type if needed
approach_client = shelf_attacing_node.create_client(GoToLoading, 'approach_shelf')

# Create a parameter client
param_client = shelf_attacing_node.create_client(
    SetParameters, 
    '/local_costmap/local_costmap/set_parameters'
)

def update_footprint_for_shelf(shelf):
    
    while not param_client.wait_for_service(timeout_sec=1.0):
        print('Parameter service not available, waiting...')
    
    if shelf:
        # Larger footprint when carrying shelf
        footprint = _footprint_with_shelf
    else:
        # RB1's original footprint
        footprint = _footprint_without_shelf

    
    # Create a simple Parameter directly
    param = Parameter(name='footprint', value=footprint)
    
    # Convert to the ROS service parameter type
    ros_param = Parameter.to_parameter_msg(param)
    
    # Create request
    request = SetParameters.Request()
    request.parameters = [ros_param]
    
    # Call service
    updated_footprint_future = param_client.call_async(request)
    rclpy.spin_until_future_complete(shelf_attacing_node, updated_footprint_future)
    
    # rcl_interfaces.srv.SetParameters_Response(results=[rcl_interfaces.msg.SetParametersResult(successful=True, reason='')])
    return updated_footprint_future.result()


def bot_attach_shelf():

    # Wait for service to be available
    while not approach_client.wait_for_service(timeout_sec=1.0):
        print('Approach service not available, waiting...')

    # Create request
    attach_shelf_request = GoToLoading.Request(attach_to_shelf = True)

    # Call service asynchronously
    attach_shelf_future = approach_client.call_async(attach_shelf_request)

    # Wait for response
    rclpy.spin_until_future_complete(shelf_attacing_node, attach_shelf_future)

    # Check result
    if not attach_shelf_future.result().complete:
        print('Move towards shelf INCOMPLETE')
        return 0

    print('Move underneath shelf and attached shelf sucessfully')
    
    update_footprint_for_shelf(shelf=True)

    return True

def move_backward_to_pose(node, target_pose, distance=1.0):
    """Move backward to target_pose by specified distance"""
    # Create velocity publisher
    cmd_vel_publisher = node.create_publisher(
        Twist, 
        '/diffbot_base_controller/cmd_vel_unstamped',
        10
    )
    
    # Create odometry listener
    odom_listener = OdomListener(node)
    
    # Wait for odometry to be available
    print("Waiting for odometry data...")
    start_time = time.time()
    while odom_listener.get_current_pose() is None:
        time.sleep(0.1)
        if time.time() - start_time > 5.0:
            print("Timeout waiting for odometry!")
            return False
    
    # Record starting position
    start_pose = odom_listener.get_current_pose()
    start_x = start_pose.position.x
    start_y = start_pose.position.y
    
    # Calculate direction vector from robot to target
    dx = target_pose.pose.position.x - start_x
    dy = target_pose.pose.position.y - start_y
    
    # Normalize direction vector
    mag = math.sqrt(dx*dx + dy*dy)
    if mag > 0:
        dx /= mag
        dy /= mag
    
    # Move backward (opposite direction)
    vel_msg = Twist()
    vel_msg.linear.x = -0.2  # Constant backward speed
    
    distance_moved = 0.0
    rate = node.create_rate(10)  # 10 Hz
    
    print("Moving backward toward loading position...")
    while distance_moved < distance:
        # Get current position
        current_pose = odom_listener.get_current_pose()
        if current_pose is None:
            cmd_vel_publisher.publish(Twist())  # Stop if odometry lost
            return False
        
        # Calculate distance moved
        current_x = current_pose.position.x
        current_y = current_pose.position.y
        distance_moved = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
        
        # Publish velocity
        cmd_vel_publisher.publish(vel_msg)
        rate.sleep()
    
    # Stop the robot
    stop_msg = Twist()
    cmd_vel_publisher.publish(stop_msg)
    time.sleep(0.5)  # Make sure stop command is processed
    
    print(f"Moved backward {distance_moved:.2f} meters")
    return True

def main():

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # Values from amcl config under localization_server
    initial_pose.pose.position.x = _initial_position[0]
    initial_pose.pose.position.y = _initial_position[1]
    
    # Convert yaw(0.00883951 from amcl config) to quaternion
    q = quaternion_from_euler(0.0, 0.0, _initial_position[2])
    initial_pose.pose.orientation.x = q[0]
    initial_pose.pose.orientation.y = q[1]
    initial_pose.pose.orientation.z = q[2]
    initial_pose.pose.orientation.w = q[3]

    # initialize loading position
    load_pose = PoseStamped()
    load_pose.header.frame_id = 'map'
    load_pose.header.stamp = navigator.get_clock().now().to_msg()
    load_pose.pose.position.x = _loading_position[0]
    load_pose.pose.position.y = _loading_position[1]
    load_pose.pose.orientation.z = _loading_position[2]
    load_pose.pose.orientation.w = _loading_position[3]
    
    # initialize shipping position
    ship_pose = PoseStamped()
    ship_pose.header.frame_id = 'map'
    ship_pose.header.stamp = navigator.get_clock().now().to_msg()
    ship_pose.pose.position.x = _shipping_position[0]
    ship_pose.pose.position.y = _shipping_position[1]
    ship_pose.pose.orientation.z = _shipping_position[2]
    ship_pose.pose.orientation.w = _shipping_position[3]

    # initialize 1st through point position
    thru1_pose = PoseStamped()
    thru1_pose.header.frame_id = 'map'
    thru1_pose.header.stamp = navigator.get_clock().now().to_msg()
    thru1_pose.pose.position.x = _throughPoint1_position[0]
    thru1_pose.pose.position.y = _throughPoint1_position[1]
    thru1_pose.pose.orientation.z = _throughPoint1_position[2]
    thru1_pose.pose.orientation.w = _throughPoint1_position[3]
    
    # Set initial pose in the navigation system
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    # Navigate to loading position
    navigator.goToPose(load_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0:
            print('Estimated time of arrival at loading position ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Reached loading position')

        _shelf_attached = bot_attach_shelf()
        
        # # TODO: fix ❌ AttributeError: 'bool' object has no attribute 'results'
        # if not _shelf_attached.results[0].successful:
        #     print('----Unable to attach the shelf, should figure out how to use retry or recovery')
        

        move_backward_to_pose(shelf_attacing_node, load_pose, distance=0.5)

        navigator.goToPose(ship_pose)
        # print('Got product from ' + request_item_location +
        #     '! Bringing product to shipping destination (' + request_destination + ')...')



        # # instead try to use goThroughPoses (_loading_position -> _throughPoint1_position -> _shipping_position)
        # while rclpy.ok():
        #     route_poses = []
        #     route_poses.append(deepcopy(load_pose))
        #     route_poses.append(deepcopy(thru1_pose))
        #     route_poses.append(deepcopy(ship_pose))
        #     navigator.goThroughPoses(route_poses)

        #     i = 0
        #     while not navigator.isTaskComplete():
        #         i = i + 1
        #         feedback = navigator.getFeedback()
        #         if feedback and i % 10 == 0:
        #             print('Estimated time to complete current route: ' + '{0:.0f}'.format(
        #                 Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
        #                 + ' seconds.')

        #             # Some failure mode, must stop since the robot is clearly stuck
        #             if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
        #                 print('Navigation has exceeded timeout of 180s, canceling the request.')
        #                 navigator.cancelTask()

        #     # If at the end of the route, go to initial postion
        #     navigator.goToPose(load_pose)

        #     result = navigator.getResult()
        #     if result == TaskResult.SUCCEEDED:
        #         print('Route complete! Restarting...')
        #     elif result == TaskResult.CANCELED:
        #         print('Security route was canceled, exiting.')
        #         exit(1)
        #     elif result == TaskResult.FAILED:
        #         print('Security route failed! Restarting from the other side...')

    elif result == TaskResult.CANCELED:
        print('Task was canceled. Returning to staging point...  ..............................')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task failed!')
        exit(-1)

    while not navigator.isTaskComplete():
        pass
    
    exit(0)


if __name__ == '__main__':
    main()