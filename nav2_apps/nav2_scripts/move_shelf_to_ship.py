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
from nav2_msgs.srv import ClearEntireCostmap

from attach_shelf.srv import GoToLoading
from geometry_msgs.msg import Twist

from odom_listener import OdomListener

# Taken from amcl config
_initial_position = [0.0306187, 0.0211561, 0.00883951]

# Obtained from `/goal_pose` [x,y,z,w] (x & y are pose, z & w are orientation)
# _loading_position = [5.7098795, -0.1307365, -0.6950259, 0.7189846]
_loading_position = [5.7098795, -0.0307365, -0.6950259, 0.7189846]
_shipping_position = [2.5659244, 1.3705197, 0.7073400, 0.7068734]
_throughPoint1_position = [2.6281423, 0.1889415, 0.7113135, 0.7028748]
_route_routine = [_shipping_position, _throughPoint1_position]

_footprint_with_shelf = "[[0.41, 0.41], [0.41, -0.41], [-0.41, -0.41], [-0.41, 0.41]]"
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
        # footprint_clearing = False
    else:
        # RB1's original footprint
        footprint = _footprint_without_shelf
        # footprint_clearing = True

    # Create parameters list
    parameters = [
        Parameter(name='footprint', value=footprint),
        # Parameter(name='footprint_clearing_enabled', value=footprint_clearing)
    ]

    # Convert to the ROS service parameter type
    ros_parameters = [Parameter.to_parameter_msg(param) for param in parameters]
    
    # Create request
    request = SetParameters.Request()
    request.parameters = ros_parameters
    
    # Call service
    updated_footprint_future = param_client.call_async(request)
    rclpy.spin_until_future_complete(shelf_attacing_node, updated_footprint_future)
    time.sleep(0.5)
    
    # rcl_interfaces.srv.SetParameters_Response(results=[rcl_interfaces.msg.SetParametersResult(successful=True, reason='')])
    return updated_footprint_future.result()

def clear_costmaps():
    
    # Clear local costmap
    local_clear_client = shelf_attacing_node.create_client(
        ClearEntireCostmap,
        'local_costmap/clear_entirely_local_costmap'
    )
    
    while not local_clear_client.wait_for_service(timeout_sec=1.0):
        print('Clear local costmap service not available, waiting...')
    
    request = ClearEntireCostmap.Request()
    future = local_clear_client.call_async(request)
    rclpy.spin_until_future_complete(shelf_attacing_node, future)
    print("Cleared local costmap")
    
    # Clear global costmap
    global_clear_client = shelf_attacing_node.create_client(
        ClearEntireCostmap,
        'global_costmap/clear_entirely_global_costmap'
    )
    
    while not global_clear_client.wait_for_service(timeout_sec=1.0):
        print('Clear global costmap service not available, waiting...')
    
    request = ClearEntireCostmap.Request()
    future = global_clear_client.call_async(request)
    rclpy.spin_until_future_complete(shelf_attacing_node, future)
    print("Cleared global costmap")
    
    return True

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

    clear_costmaps()

    return True

def move_backward_from_shelf(distance=0.5, speed=0.2):
    """Simple backward motion without turning"""
    cmd_vel_publisher = shelf_attacing_node.create_publisher(
        Twist, 
        '/diffbot_base_controller/cmd_vel_unstamped',
        10
    )
    
    print(f"Moving backward {distance} meters at {speed} m/s")
    
    # Create velocity
    vel_msg = Twist()
    vel_msg.linear.x = -speed 
    vel_msg.angular.z = 0.10
    
    # Publish for several seconds to ensure the robot moves
    start_time = time.time()
    duration = distance / speed  # time needed to move the distance
    
    print(f"Publishing backward command for {duration:.2f} seconds...")
    
    while time.time() - start_time < duration:
        cmd_vel_publisher.publish(vel_msg)
        time.sleep(0.1)

    print ('------------- curved backing up done')

    vel_msg.angular.z = -0.7854
    vel_msg.linear.x = 0.0
    start_time = time.time()
    while time.time() - start_time < 3:
        cmd_vel_publisher.publish(vel_msg)
        time.sleep(1.0)

    print('---------- rotation comlete')

    
    # Send stop multiple times to ensure
    print("Sending stop command...")
    stop_msg = Twist()
    for i in range(10):  # Send multiple stop commands
        cmd_vel_publisher.publish(stop_msg)
        time.sleep(0.1)
    
    print("Backward motion complete")
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

    # Set initial pose in the navigation system
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    # initialize loading position
    load_pose = PoseStamped()
    load_pose.header.frame_id = 'map'
    load_pose.header.stamp = navigator.get_clock().now().to_msg()
    load_pose.pose.position.x = _loading_position[0]
    load_pose.pose.position.y = _loading_position[1]
    load_pose.pose.orientation.z = _loading_position[2]
    load_pose.pose.orientation.w = _loading_position[3]

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

    if result == TaskResult.CANCELED:
        print('Task was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task failed!')
        exit(-1)
    

    elif result == TaskResult.SUCCEEDED:
        print('Reached loading position')

        _shelf_attached = bot_attach_shelf()
        
        # # TODO: fix ❌ AttributeError: 'bool' object has no attribute 'results'
        # if not _shelf_attached.results[0].successful:
        #     print('----Unable to attach the shelf, should figure out how to use retry or recovery')
        

        reverse_status = move_backward_from_shelf(distance=1.750)

        if not reverse_status:
            print('Unable to take a reverse')
            print('Task failed!')
            exit(-1)


        # initialize 1st through point position
        thru1_pose = PoseStamped()
        thru1_pose.header.frame_id = 'map'
        thru1_pose.header.stamp = navigator.get_clock().now().to_msg()
        thru1_pose.pose.position.x = _throughPoint1_position[0]
        thru1_pose.pose.position.y = _throughPoint1_position[1]
        thru1_pose.pose.orientation.z = _throughPoint1_position[2]
        thru1_pose.pose.orientation.w = _throughPoint1_position[3]
        

        print('Preparing to move to shipping location')
        # initialize shipping position
        ship_pose = PoseStamped()
        ship_pose.header.frame_id = 'map'
        ship_pose.header.stamp = navigator.get_clock().now().to_msg()
        ship_pose.pose.position.x = _shipping_position[0]
        ship_pose.pose.position.y = _shipping_position[1]
        ship_pose.pose.orientation.z = _shipping_position[2]
        ship_pose.pose.orientation.w = _shipping_position[3]

        # # Navigate to shipping position
        # navigator.goToPose(thru1_pose)

        _route_pose = []
        _pose = PoseStamped()
        _pose.header.frame_id = 'map'
        _pose.header.stamp = navigator.get_clock().now().to_msg()
        for pt in _route_routine:
            _pose.pose.position.x = pt[0]
            _pose.pose.position.y = pt[1]
            _pose.pose.orientation.z = pt[2]
            _pose.pose.orientation.w = pt[3]
            _route_pose.append(deepcopy(_pose))
        navigator.goThroughPoses(_route_pose)
    
    while not navigator.isTaskComplete():
        pass
    
    exit(0)


if __name__ == '__main__':
    main()