# Task 1
### 1. Create a new package named nav2_apps 
   ros2 pkg create --build-type ament_python nav2_apps --dependencies rclpy geometry_msgs ✅

### 2. Create move_shelf_to_ship.py 
1. Set initial position using the `Simple Commander API` to initialize the robot. ✅

Taking position from previous amcl config file
```py
 _initial_position = [0.0306187, 0.0211561, 0.00883951]
```

and convert euler to quartonion for the PoseStamped compatability
```py
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
```

2. send a first goal to the robot using the `NavigateToPose` action. ✅

Obtained Positions for required locations using `/goal_pose` topic, we get the coordinates when `2D Goal pose` through rviz is given

```
user:~$ ros2 topic echo /goal_pose
header:         # loading position (sim)
  stamp:
    sec: 4073
    nanosec: 113000000
  frame_id: map
pose:
  position:
    x: 5.5098795890808105
    y: -0.1307365894317627
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: -0.6950259623952303
    w: 0.7189846393328468
---
header:         # shipping position (sim)
  stamp:
    sec: 4367
    nanosec: 894000000
  frame_id: map
pose:
  position:
    x: 2.5659244060516357
    y: 1.370519757270813
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.7073400839371501
    w: 0.7068734014346455
---
header:         # way point 1 (sim)
  stamp:
    sec: 287
    nanosec: 277000000
  frame_id: map
pose:
  position:
    x: 2.6281423568725586
    y: 0.1889415979385376
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.7113135025986204
    w: 0.7028748829065401
---
```

3. The first goal will be located inside the loading_position (as in image). ✅

Initialize these pose and proceed with `navigator.goToPose(load_pose)` and `navigator.goToPose(ship_pose)`



### 3. Once the robot is at the loading_position it has to get underneath the shelf and activate the elevator to
carry it.
    - To move the robot underneath the shelf and lift it you will not be able to use navigation (since it will
    detect the shelf as an obstacle). For this, you can recycle the code you created for Checkpoint 9, for
    * simulation  ✅
    * real robot  ❌
    - Remember that when it has loaded the shelf, the shape of the robot will be larger (because now the
    robot is carrying the shelf). So you need to change its shape (robot footprint) in Nav2 in real time (while
    navigation is still running) to prevent the robot planning over places where the robot+shelf cannot fit. ✅

### 4. Create keepout cost map - warehouse_map_keepout_sim/real ✅


# TODO's 🚧 

on real robot (lab)
1. obtain loading & shipping pos 
2. check out the intensities of the shelfs legs
3. run the approach shelf service once directly once
    ros2 service call /approach_shelf attach_shelf/srv/GoToLoading "{attach_to_shelf: true}"
4. need