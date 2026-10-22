#include "chat2me_motion/motion_controller.hpp"

#include <cmath>
#include <iostream>
#include <sstream>

namespace chat2me_motion {

namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr int kNumMotors = 32;
// Neck indices in 32-motor array (0-indexed: 17 for yaw, 18 for pitch, 19 for roll)
constexpr int kNeckYawIndex = 17;
constexpr int kNeckPitchIndex = 18;
constexpr int kNeckRollIndex = 19;

// Minimum 5th-order polynomial trajectory (smooth jerk-free S-curve)
double smoothStep(double t) {
  if (t <= 0.0) return 0.0;
  if (t >= 1.0) return 1.0;
  return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
}

}  // namespace

MotionController::MotionController(const MotionConfig &config)
    : Node("chat2me_motion_node"), config_(config) {
  current_motor_pos_.assign(kNumMotors, 0.0);

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      config_.ros.joint_state_topic, 10,
      std::bind(&MotionController::onJointState, this, std::placeholders::_1));

  pt_control_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      config_.ros.pt_control_topic, 10);

  RCLCPP_INFO(this->get_logger(),
              "MotionController initialized with joint_state: %s, pt_control: %s",
              config_.ros.joint_state_topic.c_str(),
              config_.ros.pt_control_topic.c_str());

  worker_thread_ = std::thread(&MotionController::controlLoop, this);
}

MotionController::~MotionController() {
  running_.store(false);
  preempt_requested_.store(true);
  queue_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

double MotionController::clampYaw(double rad) const {
  const double max_rad = std::abs(config_.neck.yaw_limit_deg) * kDegToRad;
  if (rad > max_rad) return max_rad;
  if (rad < -max_rad) return -max_rad;
  return rad;
}

double MotionController::clampPitch(double rad) const {
  const double min_rad = config_.neck.pitch_min_deg * kDegToRad;
  const double max_rad = config_.neck.pitch_max_deg * kDegToRad;
  if (rad < min_rad) return min_rad;
  if (rad > max_rad) return max_rad;
  return rad;
}

double MotionController::clampRoll(double rad) const {
  const double min_rad = config_.neck.roll_min_deg * kDegToRad;
  const double max_rad = config_.neck.roll_max_deg * kDegToRad;
  if (rad < min_rad) return min_rad;
  if (rad > max_rad) return max_rad;
  return rad;
}

void MotionController::onJointState(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto size = msg->position.size();
  if (size >= kNumMotors) {
    for (size_t i = 0; i < kNumMotors; ++i) {
      current_motor_pos_[i] = msg->position[i];
    }
    cur_neck_yaw_ = msg->position[kNeckYawIndex];
    cur_neck_pitch_ = msg->position[kNeckPitchIndex];
    cur_neck_roll_ = msg->position[kNeckRollIndex];
    state_received_ = true;
  }
}

bool MotionController::enqueueAction(const ActionGoal &goal) {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (goal.preempt) {
    preempt_requested_.store(true);
    action_queue_.clear();
  }
  action_queue_.push_back(goal);
  queue_cv_.notify_one();
  return true;
}

void MotionController::sendNeckPTCommand(double yaw, double pitch, double roll,
                                        double yaw_vel, double pitch_vel,
                                        double roll_vel) {
  // Command structure: 160 elements (32*5: kp, kd, pos_des, vel_des, tor_ff)
  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(kNumMotors * 5, 0.0);

  // Set kp
  msg.data[kNeckYawIndex] = config_.neck.kp;
  msg.data[kNeckPitchIndex] = config_.neck.kp;
  msg.data[kNeckRollIndex] = config_.neck.kp;

  // Set kd
  msg.data[kNeckYawIndex + kNumMotors] = config_.neck.kd;
  msg.data[kNeckPitchIndex + kNumMotors] = config_.neck.kd;
  msg.data[kNeckRollIndex + kNumMotors] = config_.neck.kd;

  // Set pos_des
  msg.data[kNeckYawIndex + kNumMotors * 2] = clampYaw(yaw);
  msg.data[kNeckPitchIndex + kNumMotors * 2] = clampPitch(pitch);
  msg.data[kNeckRollIndex + kNumMotors * 2] = clampRoll(roll);

  // Set vel_des
  msg.data[kNeckYawIndex + kNumMotors * 3] = yaw_vel;
  msg.data[kNeckPitchIndex + kNumMotors * 3] = pitch_vel;
  msg.data[kNeckRollIndex + kNumMotors * 3] = roll_vel;

  // tor_ff remains 0.0

  pt_control_pub_->publish(msg);
}

void MotionController::controlLoop() {
  while (running_.load()) {
    ActionGoal goal;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this]() {
        return !action_queue_.empty() || !running_.load();
      });
      if (!running_.load()) break;
      goal = action_queue_.front();
      action_queue_.pop_front();
      preempt_requested_.store(false);
    }

    try {
      executeGoal(goal);
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Error executing action %s: %s",
                  goal.name.c_str(), e.what());
    }
  }
}

void MotionController::executeGoal(const ActionGoal &goal) {
  RCLCPP_INFO(this->get_logger(), "Executing motion goal: %s (id=%s)",
              goal.name.c_str(), goal.id.c_str());

  double start_yaw = 0.0;
  double start_pitch = 0.0;
  double start_roll = 0.0;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_received_) {
      start_yaw = cur_neck_yaw_;
      start_pitch = cur_neck_pitch_;
      start_roll = cur_neck_roll_;
    }
  }

  const int hz = std::max(20, config_.ros.control_hz);
  const auto dt = std::chrono::milliseconds(1000 / hz);

  auto interpolate = [&](double target_y, double target_p, double target_r,
                         double duration_sec) -> bool {
    const int steps = std::max(1, static_cast<int>(duration_sec * hz));
    const double init_y = start_yaw;
    const double init_p = start_pitch;
    const double init_r = start_roll;

    for (int i = 1; i <= steps; ++i) {
      if (preempt_requested_.load() || !running_.load()) {
        return false;
      }
      const double s = smoothStep(static_cast<double>(i) / steps);
      const double y = init_y + s * (target_y - init_y);
      const double p = init_p + s * (target_p - init_p);
      const double r = init_r + s * (target_r - init_r);
      sendNeckPTCommand(y, p, r);
      start_yaw = y;
      start_pitch = p;
      start_roll = r;
      std::this_thread::sleep_for(dt);
    }
    return true;
  };

  auto hold = [&](double hold_sec) -> bool {
    const int steps = std::max(1, static_cast<int>(hold_sec * hz));
    for (int i = 0; i < steps; ++i) {
      if (preempt_requested_.load() || !running_.load()) return false;
      sendNeckPTCommand(start_yaw, start_pitch, start_roll);
      std::this_thread::sleep_for(dt);
    }
    return true;
  };

  if (goal.name == "nod") {
    // Nod down, hold, then return
    if (!interpolate(start_yaw, 15.0 * kDegToRad, start_roll, 0.45)) return;
    if (!hold(0.1)) return;
    if (!interpolate(start_yaw, -5.0 * kDegToRad, start_roll, 0.40)) return;
    if (!interpolate(start_yaw, 0.0, 0.0, 0.35)) return;
  } else if (goal.name == "shake") {
    // Shake head left-right within ±30 deg
    const double limit = std::min(30.0, config_.neck.yaw_limit_deg) * kDegToRad;
    if (!interpolate(limit, start_pitch, start_roll, 0.35)) return;
    if (!interpolate(-limit, start_pitch, start_roll, 0.50)) return;
    if (!interpolate(limit * 0.5, start_pitch, start_roll, 0.35)) return;
    if (!interpolate(0.0, 0.0, 0.0, 0.35)) return;
  } else if (goal.name == "look_around") {
    // Look left (max ±70 deg limit), look right, then center
    const double max_yaw = config_.neck.yaw_limit_deg * kDegToRad * 0.85;
    if (!interpolate(-max_yaw, start_pitch, start_roll, 0.80)) return;
    if (!hold(0.35)) return;
    if (!interpolate(max_yaw, start_pitch, start_roll, 1.20)) return;
    if (!hold(0.35)) return;
    if (!interpolate(0.0, 0.0, 0.0, 0.80)) return;
  } else if (goal.name == "think") {
    // Tilt head (roll) and slightly pitch up
    const double tilt = 15.0 * kDegToRad;
    const double pitch_up = -8.0 * kDegToRad;
    if (!interpolate(start_yaw, pitch_up, tilt, 0.65)) return;
    if (!hold(1.2)) return;
    if (!interpolate(0.0, 0.0, 0.0, 0.70)) return;
  } else if (goal.name == "look_at") {
    // Sound direction: ReSpeaker angle 0=front, 90=right, 270=left
    // Robot yaw: positive = left, negative = right
    double target_yaw = 0.0;
    if (goal.has_direction) {
      double angle = goal.direction_deg;
      while (angle > 180.0) angle -= 360.0;
      while (angle < -180.0) angle += 360.0;
      // Convert clockwise DOA (0 front, positive right) to neck yaw (positive left)
      target_yaw = -angle * kDegToRad;
      // Clamp strictly within ±70 deg (handled by clampYaw)
      target_yaw = clampYaw(target_yaw);
    }
    if (!interpolate(target_yaw, 0.0, 0.0, 0.75)) return;
    if (!hold(1.5)) return;
    if (!interpolate(0.0, 0.0, 0.0, 0.75)) return;
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown action: %s", goal.name.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Action %s finished", goal.name.c_str());
}

}  // namespace chat2me_motion
