#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using moveit::planning_interface::MoveGroupInterface;
using moveit_visual_tools::MoveItVisualTools;

class PandaGraspDemo : public rclcpp::Node, public std::enable_shared_from_this<PandaGraspDemo>
{
public:
    PandaGraspDemo() : Node("panda_grasp_demo")
    {
        RCLCPP_INFO(this->get_logger(), "Panda Grasp Demo Node Started");

        // 创建一个定时器，0.5秒后执行initialize一次
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PandaGraspDemo::initialize, this));
    }

private:
    std::shared_ptr<MoveGroupInterface> move_group_arm_;
    std::shared_ptr<MoveGroupInterface> move_group_gripper_;
    std::shared_ptr<MoveItVisualTools> visual_tools_;
    rclcpp::TimerBase::SharedPtr timer_;

    void initialize()
    {
        // 取消定时器，保证只调用一次initialize
        timer_->cancel();

        // 这里使用shared_from_this()确保安全地获取shared_ptr
        auto self = this->shared_from_this();

        // 初始化MoveGroup接口
        move_group_arm_ = std::make_shared<MoveGroupInterface>(self, "panda_arm");
        move_group_gripper_ = std::make_shared<MoveGroupInterface>(self, "hand");

        // 初始化可视化工具
        visual_tools_ = std::make_shared<MoveItVisualTools>(self, "panda_link0", "moveit_visual_markers");
        visual_tools_->deleteAllMarkers();
        visual_tools_->loadRemoteControl();

        RCLCPP_INFO(this->get_logger(), "MoveGroup and VisualTools initialized");

        // 执行抓取动作
        execute_grasp();
    }

    void execute_grasp()
    {
        // 设置预抓取位置姿态
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = 1.0;
        target_pose.position.x = 0.4;
        target_pose.position.y = 0.0;
        target_pose.position.z = 0.4;

        // 1. 移动机械臂到预抓取位姿
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
        else
        {
            RCLCPP_WARN(this->get_logger(), "Gripper planning failed");
            return;
        }

        // 3. 抬起物体
        target_pose.position.z += 0.1;  // 抬高0.1米
        move_group_arm_->setPoseTarget(target_pose);
        if (move_group_arm_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS)
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

    // 使用enable_shared_from_this包装，保证shared_from_this()可用
    auto node = std::make_shared<PandaGraspDemo>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
