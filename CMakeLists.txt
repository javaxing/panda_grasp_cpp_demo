cmake_minimum_required(VERSION 3.8)
project(panda_grasp_cpp_demo)

# 编译选项
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(panda_grasp_demo_node src/panda_grasp_demo_node.cpp)

ament_target_dependencies(panda_grasp_demo_node
        rclcpp
        moveit_ros_planning_interface
        moveit_visual_tools
        geometry_msgs
)

install(TARGETS panda_grasp_demo_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
