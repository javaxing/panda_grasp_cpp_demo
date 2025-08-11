#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using moveit::planning_interface::MoveGroupInterface;
using moveit_visual_tools::MoveItVisualTools;

class PandaGraspDemo : public rclcpp::Node
{
public:
    PandaGraspDemo() : Node("panda_grasp_demo")
    {
        RCLCPP_INFO(this->get_logger(), "Panda Grasp Demo Node Started");
    }

    void initialize()
    {
        // 用 rclcpp::Node 自带的 shared_from_this()
        auto self = shared_from_this();

        move_group_arm_ = std::make_shared<MoveGroupInterface>(self, "panda_arm");
        move_group_gripper_ = std::make_shared<MoveGroupInterface>(self, "hand");

        visual_tools_ = std::make_shared<MoveItVisualTools>(self, "panda_link0", "moveit_visual_markers");
        visual_tools_->deleteAllMarkers();
        visual_tools_->loadRemoteControl();

        RCLCPP_INFO(this->get_logger(), "MoveGroup and VisualTools initialized");
        execute_grasp();
    }

private:
    std::shared_ptr<MoveGroupInterface> move_group_arm_;
    std::shared_ptr<MoveGroupInterface> move_group_gripper_;
    std::shared_ptr<MoveItVisualTools> visual_tools_;

    void execute_grasp()
    {
        namespace rvt = rviz_visual_tools;

        // 1. 移动到预抓取位置
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = 1.0;
        target_pose.position.x = 0.4;
        target_pose.position.y = 0.0;
        target_pose.position.z = 0.4;

        move_group_arm_->setPoseTarget(target_pose);
        MoveGroupInterface::Plan arm_plan;
        bool success = (move_group_arm_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Arm plan found, executing...");
            move_group_arm_->execute(arm_plan);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Arm planning failed");
            return;
        }

        // 2. 闭合夹爪
        move_group_gripper_->setNamedTarget("close");
        MoveGroupInterface::Plan gripper_plan;
        if (move_group_gripper_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            move_group_gripper_->execute(gripper_plan);
            RCLCPP_INFO(this->get_logger(), "Gripper closed");
        }

        // 3. 抬起物体
        target_pose.position.z += 0.1;
        move_group_arm_->setPoseTarget(target_pose);
        if (move_group_arm_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            move_group_arm_->execute(arm_plan);
            RCLCPP_INFO(this->get_logger(), "Object lifted");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PandaGraspDemo>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
