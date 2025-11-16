import time
from math import pi as PI
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
from std_msgs.msg import String

from manual_movement import ManualMover

# Taken from amcl config
_initial_position = [-0.02597, -0.0462, 0.041472, 0.99913]

# Obtained from `/amcl_pose` [x,y,z,w] (x & y are pose, z & w are orientation)
_loading_position = [4.3563, -0.7774, -0.71781, 0.69623]
_shipping_position = [1.84022, 0.94194, 0.66075, 0.750598]
_throughPoint1_position = [1.93003, -0.17741, 0.724337, 0.68944]

_footprint_with_shelf = "[[0.32, 0.32], [0.32, -0.32], [-0.32, -0.32], [-0.32, 0.32]]"
_footprint_without_shelf = "[[0.26, 0.26], [0.26, -0.26], [-0.26, -0.26], [-0.26, 0.26]]"


rclpy.init()

maneuver = ManualMover()

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

elevatorDown_pub = shelf_attacing_node.create_publisher(
    String, '/elevator_down', 10
)


def update_footprint_for_shelf(shelf):
    
    while not param_client.wait_for_service(timeout_sec=1.0):
        print('Parameter service not available, waiting...')
    
    if shelf:
        # Larger footprint when carrying shelf
        footprint = _footprint_with_shelf
        footprint_clearing = False
    else:
        # RB1's original footprint
        footprint = _footprint_without_shelf
        footprint_clearing = True

    # Create parameters list
    parameters = [
        Parameter(name='footprint', value=footprint),
        Parameter(name='footprint_clearing_enabled', value=footprint_clearing)
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

    if updated_footprint_future.result():
        if shelf:
            print('Footprint updated WITH shelf')
        else:
            print('Footprint updated WITHOUT shelf')
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

def bot_approach_attach_shelf():

    # Wait for service to be available
    while not approach_client.wait_for_service(timeout_sec=1.0):
        print('Approach service not available, waiting...')

    # Create request
    # attach_shelf_request = GoToLoading.Request(attach_to_shelf = True)
    attach_shelf_request = GoToLoading.Request(attach_to_shelf = False)

    # Call service asynchronously
    print('`/approach_shelf` service calling inititated...')
    attach_shelf_future = approach_client.call_async(attach_shelf_request)

    # Wait for response
    rclpy.spin_until_future_complete(shelf_attacing_node, attach_shelf_future)

    # Check result
    # if not attach_shelf_future.result().complete:
    #     print('Move towards shelf INCOMPLETE')
    #     return 0

    # print('Move underneath shelf and attached shelf SUCCESSFUL')
    
    # update_footprint_for_shelf(shelf=True)

    # clear_costmaps()

    # return True
    if attach_shelf_future.done():
        if attach_shelf_future.result() is not None:
            resp = attach_shelf_future.result()

            if resp.complete:
                print('Move underneath shelf and attached SUCCESSFUL')

                update_footprint_for_shelf(shelf=True)
                clear_costmaps()
                
                return True
            else:
                print('Move underneath shelf and attached FAILURE')
                
                return False

    else:
        print('approach_shelf service call FAILURE')
        if attach_shelf_future.exception():
            print(f'Exception: {attach_shelf_future.exception()}')
        
        return False


def goToLocation(position: list, action: bool = None):
    _pose_name = [name for name, value in globals().items() if value is position][0]
    print(f' >> Approaching {_pose_name}')

    _pose = PoseStamped()
    _pose.header.frame_id = 'map'
    _pose.header.stamp = navigator.get_clock().now().to_msg()
    _pose.pose.position.x = position[0]
    _pose.pose.position.y = position[1]
    try:
        _pose.pose.orientation.z = position[2]
        _pose.pose.orientation.w = position[3]
    except:
        print(f'No orientation available for {_pose_name} ')

    navigator.goToPose(_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival : {0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f' << Reached {_pose_name}')
        if action:
            print('Received action to be executed after goal reached')
            trigger_elevator()

    elif result == TaskResult.CANCELED:
        print('Task was canceled. Returning to staging point...')
        return False

    elif result == TaskResult.FAILED:
        print('Task failed!')
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    print(f' << Reached {_pose_name}')
    return True

def trigger_elevator():
    print("Triggering elevator down command...")
    msg = String()
    msg.data = "down"
    for _x in range(2):
        elevatorDown_pub.publish(msg)
        time.sleep(1.0)  # give it a moment to publish
    time.sleep(6)
    print("Elevator command sent")

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

    # amcl correction
    # print('-- Just ensuring proper Localization --')
    # maneuver.inplace_rotation(rotate_deg=90, rotate_speed=0.2)
    # time.sleep(0.5)
    # maneuver.inplace_rotation(rotate_deg=-180, rotate_speed=0.4)
    # time.sleep(0.5)
    # maneuver.inplace_rotation(rotate_deg=90, rotate_speed=0.2)
    # time.sleep(0.5)

    # initialize loading position
    load_pose = PoseStamped()
    load_pose.header.frame_id = 'map'
    load_pose.header.stamp = navigator.get_clock().now().to_msg()
    load_pose.pose.position.x = _loading_position[0]
    load_pose.pose.position.y = _loading_position[1]
    load_pose.pose.orientation.z = _loading_position[2]
    load_pose.pose.orientation.w = _loading_position[3]

    # Navigate to loading position
    print('Received initial goal position and started moving')
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

        _shelf_attached = bot_approach_attach_shelf()

    while not navigator.isTaskComplete():
        time.sleep(0.5)
        print('Navigator GoToLoading COMPLETED')

    
    if _shelf_attached:

        maneuver.move_backward(distance=1.40, curvature_deg=6.0, speed=0.3)
        time.sleep(0.5)
        
        maneuver.inplace_rotation(rotate_deg=-180, rotate_speed=0.5)
        time.sleep(0.5)

        goToLocation(position=_throughPoint1_position)

        goToLocation(position=_shipping_position, action=True)

        update_footprint_for_shelf(shelf=False)

        maneuver.move_backward(distance=1.4, speed=0.3)
        time.sleep(0.5)

        clear_costmaps()

        maneuver.inplace_rotation(rotate_deg=90, rotate_speed=0.5)
        time.sleep(0.5)

    else:
        print('Attach shelf FAILURE. Returing to initial position')

    goToLocation(position=_initial_position)

    while not navigator.isTaskComplete():
        pass

    exit(0)

if __name__ == '__main__':
    main()
