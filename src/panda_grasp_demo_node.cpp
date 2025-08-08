#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PandaGraspDemo : public rclcpp::Node
{
public:
  PandaGraspDemo()
      : Node("panda_grasp_demo")
  {
    RCLCPP_INFO(this->get_logger(), "Panda Grasp Demo Node Started");

    // 正确获取 NodeInterface，用于 MoveGroupInterface 初始化
    move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        this->shared_from_this(), "panda_arm");

    move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        this->shared_from_this(), "hand");

    // 设置规划参考坐标系
    move_group_arm_->setPoseReferenceFrame("world");

    // 执行抓取流程
    openGripper();
    moveToPreGraspPose();
    closeGripper();
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

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      move_group_arm_->execute(plan);
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
    current_pose.position.z += 0.1;

    move_group_arm_->setPoseTarget(current_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      move_group_arm_->execute(plan);
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
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_gripper_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      move_group_gripper_->execute(plan);
      RCLCPP_INFO(this->get_logger(), "Opened gripper.");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open gripper.");
    }
  }

  void closeGripper()
  {
    move_group_gripper_->setNamedTarget("close");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_gripper_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      move_group_gripper_->execute(plan);
      RCLCPP_INFO(this->get_logger(), "Closed gripper.");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to close gripper.");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // 为使 shared_from_this 可用，需先使用 enable_shared_from_this 包装
  auto node = std::make_shared<PandaGraspDemo>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
