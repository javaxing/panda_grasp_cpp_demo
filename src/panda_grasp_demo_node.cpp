#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "panda_grasp_cpp_demo/msg/robot_status.hpp"          // 话题消息
#include "panda_grasp_cpp_demo/srv/move_to_target.hpp"       // 服务
#include "panda_grasp_cpp_demo/action/navigate.hpp"          // 动作

#include <memory>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

// 定义节点类
class RobotDemoNode : public rclcpp::Node {
public:
    RobotDemoNode() : Node("panda_grasp_cpp_demo_node") {
        // ======================
        // 1️⃣ 参数（Parameter）
        // ======================
        this->declare_parameter("max_speed", 1.0);
        this->declare_parameter("debug_mode", true);

        max_speed_ = this->get_parameter("max_speed").as_double();
        debug_mode_ = this->get_parameter("debug_mode").as_bool();

        RCLCPP_INFO(this->get_logger(), "机器人启动！最大速度: %.2f, 调试模式: %s",
                    max_speed_, debug_mode_ ? "开启" : "关闭");

        // ======================
        // 2️⃣ 话题（Topic）：发布机器人状态
        // ======================
        status_pub_ = this->create_publisher<RobotStatus>("robot_status", 10);

        // 定时发布状态（模拟传感器数据）
        timer_ = this->create_wall_timer(
            1s, std::bind(&RobotDemoNode::publish_status, this));

        // ======================
        // 3️⃣ 服务（Service）：处理移动请求（同步）
        // ======================
        move_service_ = this->create_service<MoveToTarget>(
            "move_to_target",
            std::bind(&RobotDemoNode::handle_move_request, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ======================
        // 4️⃣ 动作（Action）：处理导航任务（带反馈）
        // ======================
        navigate_action_ = rclcpp_action::create_server<Navigate>(
            this,
            "navigate_to_goal",
            std::bind(&RobotDemoNode::handle_navigate_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RobotDemoNode::handle_navigate_cancel, this, std::placeholders::_1),
            std::bind(&RobotDemoNode::handle_navigate_accept, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "ROS 2 四种通信方式已启动！");
    }

private:
    // 参数
    double max_speed_;
    bool debug_mode_;

    // 话题：发布状态
    rclcpp::Publisher<RobotStatus>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 服务：移动目标
    rclcpp::Service<MoveToTarget>::SharedPtr move_service_;

    // 动作：导航
    rclcpp_action::Server<Navigate>::SharedPtr navigate_action_;

    // 发布机器人状态
    void publish_status() {
        auto msg = RobotStatus();
        msg.status = debug_mode_ ? "运行中" : "正常";
        msg.x = 0.0;  // 模拟值
        msg.y = 0.0;
        msg.speed = max_speed_;
        status_pub_->publish(msg);
    }

    // 服务回调：移动到目标点
    void handle_move_request(
        const MoveToTarget::Request::SharedPtr request,
        MoveToTarget::Response::SharedPtr response) {
        RCLCPP_INFO(this->get_logger(), "收到移动请求 -> 目标: (%.2f, %.2f)", request->x, request->y);

        // 模拟处理
        std::this_thread::sleep_for(1s);  // 模拟耗时操作

        response->success = true;
        response->message = "已移动到目标点！";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    // ========== 动作相关回调 ==========

    rclcpp_action::GoalResponse handle_navigate_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Navigate::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "收到导航目标 -> (%.2f, %.2f)", goal->x, goal->y);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_navigate_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Navigate>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "导航任务取消请求");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_navigate_accept(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Navigate>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "执行导航任务...");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Navigate::Feedback>();
        auto result = std::make_shared<Navigate::Result>();

        // 模拟导航过程，每秒反馈一次进度
        for (int i = 0; i <= 10; ++i) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "用户取消";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "导航已取消");
                return;
            }

            feedback->progress = i * 0.1f;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "导航进度: %.0f%%", feedback->progress * 100.0f);
            std::this_thread::sleep_for(1s);
        }

        result->success = true;
        result->message = "到达目标！";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDemoNode>());
    rclcpp::shutdown();
    return 0;
}