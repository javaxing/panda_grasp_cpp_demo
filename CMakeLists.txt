cmake_minimum_required(VERSION 3.8)
project(panda_grasp_cpp_demo)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
# 在 find_package 部分确保有以下内容
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)


# 修改接口生成部分为
rosidl_generate_interfaces(${PROJECT_NAME}
        ${rosidl_generate_interfaces_IDL_FILES}
        "msg/RobotStatus.msg"
        "srv/MoveToTarget.srv"
        "action/Navigate.action"
        DEPENDENCIES std_msgs geometry_msgs
)

# Create executable
add_executable(panda_grasp_demo_node src/panda_grasp_demo_node.cpp)

# 关键链接配置
target_include_directories(panda_grasp_demo_node PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
        ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_cpp/include
)

target_link_libraries(panda_grasp_demo_node
        ${PROJECT_NAME}__rosidl_generator_cpp
        ${PROJECT_NAME}__rosidl_typesupport_cpp
)
ament_target_dependencies(panda_grasp_demo_node
        rclcpp
        std_msgs
        geometry_msgs
        rclcpp_action
        rosidl_default_runtime
        rosidl_typesupport_cpp
)

# Install
install(DIRECTORY
        ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/
        ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_cpp/
        DESTINATION include/${PROJECT_NAME}
)

ament_package()