#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;
using moveit_visual_tools::MoveItVisualTools;

class PandaGraspDemo : public rclcpp::Node
{
public:
    PandaGraspDemo() : Node("panda_grasp_demo")
    {
        RCLCPP_INFO(this->get_logger(), "Panda Grasp Demo Node Started");

        // 用定时器延迟初始化，确保 shared_from_this() 有效
        timer_ = this->create_wall_timer(
            500ms, std::bind(&PandaGraspDemo::initialize, this));
    }

private:
    std::shared_ptr<MoveGroupInterface> move_group_arm_;
    std::shared_ptr<MoveGroupInterface> move_group_gripper_;
    std::shared_ptr<MoveItVisualTools> visual_tools_;
    rclcpp::TimerBase::SharedPtr timer_;

    void initialize()
    {
        // 取消定时器，保证初始化只执行一次
        timer_->cancel();

        // 显式调用 rclcpp::Node 的 shared_from_this()
        auto self = rclcpp::Node::shared_from_this();

        move_group_arm_ = std::make_shared<MoveGroupInterface>(self, "panda_arm");
        move_group_gripper_ = std::make_shared<MoveGroupInterface>(self, "hand");

        visual_tools_ = std::make_shared<MoveItVisualTools>(self, "panda_link0", "moveit_visual_markers");
        visual_tools_->deleteAllMarkers();
        visual_tools_->loadRemoteControl();

        RCLCPP_INFO(this->get_logger(), "MoveGroup and VisualTools initialized");

        execute_grasp();
    }

    void execute_grasp()
    {
        // 1. 移动机械臂到预抓取位置
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = 1.0;
        target_pose.position.x = 0.4;
        target_pose.position.y = 0.0;
        target_pose.position.z = 0.4;

        move_group_arm_->setPoseTarget(target_pose);

        MoveGroupInterface::Plan arm_plan;
        bool success = (move_group_arm_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            RCLCPP_WARN(this->get_logger(), "Arm planning failed");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Arm plan found, executing...");
        move_group_arm_->execute(arm_plan);

        // 2. 关闭夹爪（抓取）
        move_group_gripper_->setNamedTarget("close");
        MoveGroupInterface::Plan gripper_plan;
        success = (move_group_gripper_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            move_group_gripper_->execute(gripper_plan);
            RCLCPP_INFO(this->get_logger(), "Gripper closed");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Gripper planning failed");
            return;
        }

        // 3. 抬起机械臂
        target_pose.position.z += 0.1;
        move_group_arm_->setPoseTarget(target_pose);

        success = (move_group_arm_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            move_group_arm_->execute(arm_plan);
            RCLCPP_INFO(this->get_logger(), "Object lifted");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Lift planning failed");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 一定要用 std::make_shared 创建节点，支持 shared_from_this()
    auto node = std::make_shared<PandaGraspDemo>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
