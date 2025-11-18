cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# find_package(ament_cmake REQUIRED)
find_package(robot_config REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
)

# Utils Library (for frame & topic helpers)
# include_directories(include)
add_library(utils src/utils.cpp)
target_include_directories(
	utils 
	PUBLIC 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(utils rclcpp tf2_ros)


add_executable(pre_approach src/pre_approach.cpp)
ament_target_dependencies(pre_approach rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs )

add_executable(pre_approach_v2 src/pre_approach_v2.cpp)
ament_target_dependencies(
	pre_approach_v2 
	rclcpp 
	std_msgs
	sensor_msgs 
	geometry_msgs 
	nav_msgs 
	tf2 
	tf2_ros 
	tf2_geometry_msgs )

add_executable(approach_service_server src/approach_service_server.cpp)
ament_target_dependencies(approach_service_server 
	rclcpp 
	sensor_msgs 
	geometry_msgs 
	nav_msgs 
	std_msgs 
	tf2 
	tf2_ros
	tf2_geometry_msgs )

# Link the custom service to the executables
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(pre_approach_v2 "${cpp_typesupport_target}")
target_link_libraries(approach_service_server "${cpp_typesupport_target}")


# Link utils library
target_link_libraries(pre_approach utils)				# utils not implemented yet
target_link_libraries(pre_approach_v2 utils)			# utils not implemented yet
target_link_libraries(approach_service_server utils)	# utils implemented

# Install utils library
install(TARGETS utils
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # only if needed for Windows
)

install(TARGETS
   pre_approach
   pre_approach_v2
   approach_service_server
   DESTINATION lib/${PROJECT_NAME}
 )

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
