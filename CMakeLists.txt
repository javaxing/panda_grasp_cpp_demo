cmake_minimum_required(VERSION 3.8)
project(panda_grasp_cpp_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 接口生成
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/RobotStatus.msg"
        "srv/MoveToTarget.srv"
        "action/Navigate.action"
        DEPENDENCIES std_msgs geometry_msgs
)

add_executable(panda_grasp_demo_node src/panda_grasp_demo_node.cpp)

ament_target_dependencies(panda_grasp_demo_node
        rclcpp
        std_msgs
        geometry_msgs
        rclcpp_action
        rosidl_default_runtime
)

install(TARGETS
        panda_grasp_demo_node
        DESTINATION lib/${PROJECT_NAME}
)

# 安装 msg/srv/action
install(DIRECTORY
        msg
        srv
        action
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
