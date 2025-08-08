cmake_minimum_required(VERSION 3.5)
project(panda_grasp_cpp_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(panda_grasp_demo_node src/panda_grasp_demo_node.cpp)
ament_target_dependencies(panda_grasp_demo_node
        rclcpp std_msgs
        moveit_ros_planning_interface
        moveit_visual_tools)

install(TARGETS panda_grasp_demo_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
