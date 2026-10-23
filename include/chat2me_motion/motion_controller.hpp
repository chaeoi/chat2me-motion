#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "chat2me_motion/motion_engine.hpp"

namespace chat2me_motion {

class MotionController : public rclcpp::Node {
 public:
  explicit MotionController(const MotionConfig &config);
  MotionEngine &engine() { return engine_; }

 private:
  MotionConfig config_;
  MotionEngine engine_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace chat2me_motion
