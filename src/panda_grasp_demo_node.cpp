#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

// 继承 enable_shared_from_this 以安全使用 shared_from_this()
class PandaGraspDemo : public rclcpp::Node, public std::enable_shared_from_this<PandaGraspDemo>
{
public:
  PandaGraspDemo()
      : Node("panda_grasp_demo")
  {
    RCLCPP_INFO(this->get_logger(), "Panda Grasp Demo Node Started");
    // 不要在构造函数中调用 shared_from_this()！
    // 延迟初始化
  }

  void initialize()
  {
    // 构造之后，才能安全地调用 shared_from_this()
    move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "panda_arm");

    move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "hand");

    move_group_arm_->setPoseReferenceFrame("world");

    // 执行抓取任务
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

    if (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
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

    if (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
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

    if (move_group_gripper_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
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

    if (move_group_gripper_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
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

  // 使用 make_shared 创建智能指针实例
  auto node = std::make_shared<PandaGraspDemo>();

  // 构造完成后再调用 shared_from_this() 安全执行初始化
  node->initialize();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
