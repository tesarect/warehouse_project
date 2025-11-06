import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Taken from amcl config
_initial_position = [0.0306187, 0.0211561, 0.00883951]

# Obtained from `/goal_pose` [x,y,z,w] (x & y are pose, z & w are orientation)
_loading_position = [5.5304708, 0.0365260, -0.701839, 0.7123350]
_shipping_position = [2.5659244, 1.04464542, 0.7073400, 0.7068734]

def main():

    rclpy.init()

    navigator = BasicNavigator()

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
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at loading position ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Reached loading position.')
        # print('Got product from ' + request_item_location +
        #     '! Bringing product to shipping destination (' + request_destination + ')...')

        navigator.goToPose(ship_pose)

    elif result == TaskResult.CANCELED:
        print('Task was canceled. Returning to staging point...')
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