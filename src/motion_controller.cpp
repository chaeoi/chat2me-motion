#include "chat2me_motion/motion_controller.hpp"

#include <chrono>

namespace chat2me_motion {
namespace {
double nowSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

MotionController::MotionController(const MotionConfig &config)
    : Node("chat2me_motion_node"), config_(config), engine_(config) {
  feedback_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      config_.feedback_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        engine_.updateFeedback(msg->position, nowSeconds());
      });
  command_pub_ = create_publisher<sensor_msgs::msg::JointState>(config_.command_topic, 1);
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / config_.control_hz), [this]() {
    auto command = engine_.tick(nowSeconds());
    if (!command) return;
    const auto packed = packCommand(*command, config_);
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name.assign(packed.names.begin(), packed.names.end());
    msg.position.assign(packed.position.begin(), packed.position.end());
    msg.velocity.assign(packed.velocity.begin(), packed.velocity.end());
    msg.effort.assign(packed.effort.begin(), packed.effort.end());
    command_pub_->publish(msg);
  });
  RCLCPP_INFO(get_logger(), "Head/arm commands: %s; waiting for feedback and an action request",
              config_.command_topic.c_str());
}

}  // namespace chat2me_motion
