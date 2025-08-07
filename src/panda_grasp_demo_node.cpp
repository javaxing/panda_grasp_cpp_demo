//
// Created by xingxiaochi on 2025/8/7.
//

#include "panda_grasp_demo_node.h"

#include "rclcpp/rclcpp.hpp"

class PandaGraspDemo : public rclcpp::Node
{
public:
    PandaGraspDemo() : Node("panda_grasp_demo")
    {
        RCLCPP_INFO(this->get_logger(), "Panda Grasp Demo Node Started");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PandaGraspDemo>());
    rclcpp::shutdown();
    return 0;
}
