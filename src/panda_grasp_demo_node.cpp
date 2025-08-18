#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "panda_grasp_cpp_demo/msg/robot_status.hpp"
#include "panda_grasp_cpp_demo/srv/move_to_target.hpp"
#include "panda_grasp_cpp_demo/action/navigate.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class PandaDemoNode : public rclcpp::Node {
public:
  PandaDemoNode() : Node("panda_grasp_demo_node") {
    // ======================
    // 1. 参数 (Parameters)
    // ======================
    this->declare_parameter("max_speed", 1.0);
    this->declare_parameter("debug_mode", false);
    max_speed_ = this->get_parameter("max_speed").as_double();
    debug_mode_ = this->get_parameter("debug_mode").as_bool();

    RCLCPP_INFO(this->get_logger(), "Parameters loaded - Max Speed: %.2f, Debug Mode: %s",
                max_speed_, debug_mode_ ? "ON" : "OFF");

    // ======================
    // 2. 话题 (Topic)
    // ======================
    status_publisher_ = this->create_publisher<panda_grasp_cpp_demo::msg::RobotStatus>(
      "robot_status", 10);

    timer_ = this->create_wall_timer(
      1s, std::bind(&PandaDemoNode::publish_status, this));

    // ======================
    // 3. 服务 (Service)
    // ======================
    move_service_ = this->create_service<panda_grasp_cpp_demo::srv::MoveToTarget>(
      "move_to_target",
      std::bind(&PandaDemoNode::handle_move_request, this,
                std::placeholders::_1, std::placeholders::_2));

    // ======================
    // 4. 动作 (Action)
    // ======================
    action_server_ = rclcpp_action::create_server<panda_grasp_cpp_demo::action::Navigate>(
      this,
      "navigate_to_goal",
      std::bind(&PandaDemoNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PandaDemoNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&PandaDemoNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Panda Demo Node has started with all communication types!");
  }

private:
  // 参数变量
  double max_speed_;
  bool debug_mode_;

  // 话题相关
  rclcpp::Publisher<panda_grasp_cpp_demo::msg::RobotStatus>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 服务相关
  rclcpp::Service<panda_grasp_cpp_demo::srv::MoveToTarget>::SharedPtr move_service_;

  // 动作相关
  rclcpp_action::Server<panda_grasp_cpp_demo::action::Navigate>::SharedPtr action_server_;

  // 发布机器人状态
  void publish_status() {
    auto msg = panda_grasp_cpp_demo::msg::RobotStatus();
    msg.status = "Running";
    msg.x = 0.0;  // 实际应用中这里应该是真实坐标
    msg.y = 0.0;
    msg.speed = max_speed_;
    status_publisher_->publish(msg);

    if (debug_mode_) {
      RCLCPP_INFO(this->get_logger(), "Publishing status: x=%.2f, y=%.2f, speed=%.2f",
                  msg.x, msg.y, msg.speed);
    }
  }

  // 处理移动请求
  void handle_move_request(
    const std::shared_ptr<panda_grasp_cpp_demo::srv::MoveToTarget::Request> request,
    std::shared_ptr<panda_grasp_cpp_demo::srv::MoveToTarget::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Received move request to (%.2f, %.2f)",
                request->x, request->y);

    // 模拟移动过程
    std::this_thread::sleep_for(2s);

    response->success = true;
    response->message = "Successfully moved to target";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  // 动作服务器回调
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const panda_grasp_cpp_demo::action::Navigate::Goal> goal) {

    RCLCPP_INFO(this->get_logger(), "Received navigation goal: (%.2f, %.2f)",
                goal->target_x, goal->target_y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<panda_grasp_cpp_demo::action::Navigate>> goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<panda_grasp_cpp_demo::action::Navigate>> goal_handle) {

    std::thread{std::bind(&PandaDemoNode::execute_navigation, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute_navigation(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<panda_grasp_cpp_demo::action::Navigate>> goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Executing navigation...");
    auto feedback = std::make_shared<panda_grasp_cpp_demo::action::Navigate::Feedback>();
    auto result = std::make_shared<panda_grasp_cpp_demo::action::Navigate::Result>();

    const auto goal = goal_handle->get_goal();
    for (int i = 1; i <= 10; ++i) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Navigation canceled";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Navigation canceled");
        return;
      }

      feedback->progress = i * 0.1f;
      feedback->current_status = "Moving to target...";
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Navigation progress: %.0f%%", feedback->progress * 100.0f);

      std::this_thread::sleep_for(1s);
    }

    result->success = true;
    result->message = "Navigation completed successfully";
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Navigation completed");
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PandaDemoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}