#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "chat2me_motion/config.hpp"

namespace chat2me_motion {

struct ActionGoal {
  std::string id;
  std::string name;
  double direction_deg = 0.0;
  bool has_direction = false;
  int timeout_ms = 5000;
  bool preempt = true;
};

class MotionController : public rclcpp::Node {
 public:
  explicit MotionController(const MotionConfig &config);
  ~MotionController() override;

  bool enqueueAction(const ActionGoal &goal);

 private:
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void controlLoop();
  void executeGoal(const ActionGoal &goal);

  double clampYaw(double rad) const;
  double clampPitch(double rad) const;
  double clampRoll(double rad) const;

  void sendNeckPTCommand(double yaw, double pitch, double roll,
                         double yaw_vel = 0.0, double pitch_vel = 0.0,
                         double roll_vel = 0.0);

  MotionConfig config_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pt_control_pub_;

  std::mutex state_mutex_;
  bool state_received_{false};
  std::vector<double> current_motor_pos_;
  double cur_neck_yaw_{0.0};
  double cur_neck_pitch_{0.0};
  double cur_neck_roll_{0.0};

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<ActionGoal> action_queue_;
  std::atomic<bool> running_{true};
  std::atomic<bool> preempt_requested_{false};
  std::thread worker_thread_;
};

}  // namespace chat2me_motion
