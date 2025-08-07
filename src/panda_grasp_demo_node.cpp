//
// Created by xingxiaochi on 2025/8/7.
//

#include "panda_grasp_demo_node.h"

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class PandaGraspDemo : public rclcpp::Node
{
public:
    PandaGraspDemo() : Node("panda_grasp_demo")
    {
        RCLCPP_INFO(this->get_logger(), "Panda Grasp Demo Node Started");

        using moveit::planning_interface::MoveGroupInterface;

        // 创建 move_group 对象，用于控制 Panda 机械臂
        auto move_group = std::make_shared<MoveGroupInterface>(shared_from_this(), "panda_arm");

        // 设置目标位置
        move_group->setPoseTarget(getTargetPose());

        // 规划并执行
        auto success = (move_group->move() == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(this->get_logger(), "Move %s", success ? "SUCCESS" : "FAILED");
    }

    geometry_msgs::msg::Pose getTargetPose()
    {
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = 1.0;
        target_pose.position.x = 0.3;
        target_pose.position.y = 0.0;
        target_pose.position.z = 0.5;
        return target_pose;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PandaGraspDemo>());
    rclcpp::shutdown();
    return 0;
}
