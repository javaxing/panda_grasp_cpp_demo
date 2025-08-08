//
// Created by xingxiaochi on 2025/8/7.
//

#include "panda_grasp_demo_node.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PandaGraspDemo : public rclcpp::Node
{
public:
  PandaGraspDemo() : Node("panda_grasp_demo")
  {
    RCLCPP_INFO(this->get_logger(), "Panda Grasp Demo Node Started");

    // 1. 初始化 MoveGroupInterface（机械臂与夹爪）
    move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "panda_arm");
    move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "hand");

    // 2. 设定规划目标参考系
    move_group_arm_->setPoseReferenceFrame("world");

    // 3. 打开夹爪
    openGripper();

    // 4. 移动到预抓取姿态
    moveToPreGraspPose();

    // 5. 模拟抓取
    closeGripper();

    // 6. 提起物体
    liftObject();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;

  void moveToPreGraspPose()
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.4;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.3;

    move_group_arm_->setPoseTarget(target_pose);

    auto plan = move_group_arm_->plan();
    if (plan)
    {
      move_group_arm_->execute(*plan);
      RCLCPP_INFO(this->get_logger(), "Moved to pre-grasp pose.");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan to pre-grasp pose.");
    }
  }

  void liftObject()
  {
    auto current_pose = move_group_arm_->getCurrentPose().pose;
    current_pose.position.z += 0.1;  // 向上移动 10cm

    move_group_arm_->setPoseTarget(current_pose);

    auto plan = move_group_arm_->plan();
    if (plan)
    {
      move_group_arm_->execute(*plan);
      RCLCPP_INFO(this->get_logger(), "Lifted the object.");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to lift object.");
    }
  }

  void openGripper()
  {
    move_group_gripper_->setNamedTarget("open");
    auto plan = move_group_gripper_->plan();
    if (plan)
    {
      move_group_gripper_->execute(*plan);
      RCLCPP_INFO(this->get_logger(), "Opened gripper.");
    }
  }

  void closeGripper()
  {
    move_group_gripper_->setNamedTarget("close");
    auto plan = move_group_gripper_->plan();
    if (plan)
    {
      move_group_gripper_->execute(*plan);
      RCLCPP_INFO(this->get_logger(), "Closed gripper.");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PandaGraspDemo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
