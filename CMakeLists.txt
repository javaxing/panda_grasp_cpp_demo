cmake_minimum_required(VERSION 3.8)
project(panda_grasp_cpp_demo)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
# 必须包含的依赖
find_package(rosidl_default_generators REQUIRED)

# 接口生成部分
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/RobotStatus.msg"   # 确保路径完全正确
        "srv/MoveToTarget.srv"
        "action/Navigate.action"
)

# Create executable
add_executable(panda_grasp_demo_node src/panda_grasp_demo_node.cpp)
ament_target_dependencies(panda_grasp_demo_node
        rclcpp
        std_msgs
        geometry_msgs
        rclcpp_action
)

# Install
install(TARGETS panda_grasp_demo_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()