cmake_minimum_required(VERSION 3.8)
project(fastbot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

## ACTION INTERFACE
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WaypointAction.action"
  DEPENDENCIES geometry_msgs
)

## LIBRARIES / EXECUTABLES
add_executable(fastbot_action_server src/fastbot_action_server.cpp)
ament_target_dependencies(fastbot_action_server
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
)

# Link generated interfaces to executable
rosidl_target_interfaces(fastbot_action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  fastbot_action_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY action
  DESTINATION share/${PROJECT_NAME}/
)

## GTESTS
if(BUILD_TESTING)
  ament_add_gtest(${PROJECT_NAME}_test
    test/main.cpp
    test/waypoints_test_ros_as.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    rclcpp_action
    std_msgs
    geometry_msgs
    nav_msgs
	tf2
  )
  rosidl_target_interfaces(${PROJECT_NAME}_test
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

ament_package()
