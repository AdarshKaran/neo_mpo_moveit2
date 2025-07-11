cmake_minimum_required(VERSION 3.5)
project(neo_ur_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(gripper_controllers REQUIRED)

install(DIRECTORY config launch rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
ament_python_install_module(${PROJECT_NAME}/launch_common.py)

add_executable(mtc_pick_place_node src/mtc_pick_place_node.cpp)

ament_target_dependencies(mtc_pick_place_node 
  rclcpp 
  rclcpp_action
  moveit_task_constructor_core
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
  shape_msgs
  moveit_msgs
  control_msgs
  gripper_controllers
)

install(TARGETS mtc_pick_place_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()