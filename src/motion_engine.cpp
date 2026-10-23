#include "chat2me_motion/motion_engine.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace chat2me_motion {
namespace {
constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr std::array<const char *, kJointCount> kNames{
    "19_left_shoulder_pitch", "20_left_shoulder_roll", "21_left_shoulder_yaw",
    "22_left_elbow_pitch", "23_left_wrist_yaw", "24_left_wrist_pitch", "25_left_wrist_roll",
    "26_right_shoulder_pitch", "27_right_shoulder_roll", "28_right_shoulder_yaw",
    "29_right_elbow_pitch", "30_right_wrist_yaw", "31_right_wrist_pitch", "32_right_wrist_roll",
    "16_neck_yaw", "17_neck_pitch", "18_neck_roll"};
}

PackedCommand packCommand(const JointCommand &command, const MotionConfig &config) {
  PackedCommand packed;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    packed.names[i] = kNames[i];
    packed.position[i] = command.position[i];
    packed.position[kJointCount + i] = config.kp[i];
    packed.velocity[i] = command.velocity[i];
    packed.velocity[kJointCount + i] = config.kd[i];
  }
  return packed;
}

MotionEngine::MotionEngine(MotionConfig config) : config_(std::move(config)) {
  validateConfig(config_);
}

bool MotionEngine::fresh(double now) const {
  return feedback_time_ && now >= *feedback_time_ && now - *feedback_time_ <= config_.feedback_timeout;
}

void MotionEngine::updateFeedback(const std::vector<double> &positions, double now) {
  if (positions.size() != 32 || !std::isfinite(now)) return;
  JointVector measured;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    measured[i] = positions[3 + i];
    if (!std::isfinite(measured[i])) return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (feedback_time_ && !fresh(now)) {
    publishing_ = false;
    plan_.clear();
    command_.velocity.fill(0);
    state_ = "waiting_action";
  }
  if (!feedback_time_ || state_ == "waiting_feedback") state_ = "waiting_action";
  measured_ = measured;
  feedback_time_ = now;
}

std::vector<std::string> MotionEngine::supportedActions() const {
  std::vector<std::string> names{"nod", "shake", "look_around", "think", "look_at", "move_joints"};
  for (const auto &[name, steps] : config_.actions) {
    (void)steps;
    names.push_back(name);
  }
  return names;
}

std::vector<Waypoint> MotionEngine::waypoints(const ActionGoal &goal, const JointVector &start) const {
  if (goal.name == "move_joints") {
    if (std::none_of(goal.targets.begin(), goal.targets.end(), [](const auto &v) { return v.has_value(); }))
      throw RequestError(422, "move_joints requires left_arm, right_arm, or head targets in params");
    return {{goal.targets, goal.duration}};
  }
  auto configured = config_.actions.find(goal.name);
  if (configured != config_.actions.end()) return configured->second;

  std::vector<Waypoint> steps;
  auto step = [&](double duration) -> Waypoint & {
    steps.push_back({{}, duration});
    return steps.back();
  };
  auto target = [&](Waypoint &point, std::size_t joint, double value) {
    point.targets[joint] = std::clamp(value, config_.lower[joint], config_.upper[joint]);
  };
  auto arm = [&](Waypoint &point, std::size_t offset, const std::array<double, 7> &degrees) {
    if (!config_.arm_gestures) return;
    for (std::size_t i = 0; i < degrees.size(); ++i)
      target(point, offset + i, start[offset + i] + degrees[i] * kDeg);
  };
  // The first neck joint is Z-axis yaw. Do not reset the other axes during yaw motions.
  auto yaw = [&](double angle, double duration) {
    target(step(duration), kHeadYaw, angle);
  };
  auto gesture = [&](double duration, std::size_t offset, const std::array<double, 7> &degrees,
                     double pitch_degrees, double roll_degrees = 0) {
    auto &point = step(duration);
    target(point, kHeadPitch, start[kHeadPitch] + pitch_degrees * kDeg);
    target(point, kHeadRoll, start[kHeadRoll] + roll_degrees * kDeg);
    arm(point, offset, degrees);
  };
  const double y = start[kHeadYaw];
  if (goal.name == "nod") {
    if (!config_.arm_gestures) {
      gesture(.45, 0, {}, 15);
      gesture(.1, 0, {}, 15);
      gesture(.4, 0, {}, -5);
      gesture(.45, 0, {}, 0);
      return steps;
    }
    // Shoulder pitch/roll/yaw, elbow pitch, wrist yaw/pitch/roll, in degrees.
    // Open the shoulder before lifting; close it after the forearm is lowered.
    gesture(1.0, 0, {0, 10, 0, 0, 0, 0, 0}, 0);
    gesture(2.05, 0, {-18, 14, -10, -30, 10, -8, 20}, -3);
    gesture(1.2, 0, {-28, 20, -18, -45, 20, -12, 35}, 12);
    gesture(.75, 0, {-25, 18, -10, -40, 12, -5, 25}, -4);
    gesture(.4, 0, {-25, 18, -10, -40, 12, -5, 25}, 9);
    gesture(1.15, 0, {-15, 12, -5, -25, 5, 0, 12}, 0);
    gesture(1.9, 0, {0, 10, 0, 0, 0, 0, 0}, 0);
    gesture(1.0, 0, {}, 0);
  } else if (goal.name == "shake") {
    yaw(y + 30 * kDeg, .35);
    yaw(y - 30 * kDeg, .5);
    yaw(y + 15 * kDeg, .35);
    yaw(y, .35);
  } else if (goal.name == "look_around") {
    yaw(config_.lower[kHeadYaw] * .85, .8);
    yaw(config_.lower[kHeadYaw] * .85, .35);
    yaw(config_.upper[kHeadYaw] * .85, 1.2);
    yaw(config_.upper[kHeadYaw] * .85, .35);
    yaw(y, .8);
  } else if (goal.name == "think") {
    if (!config_.arm_gestures) {
      gesture(.65, 7, {}, -8, 15);
      gesture(1.2, 7, {}, -8, 15);
      gesture(.7, 7, {}, 0);
      return steps;
    }
    gesture(1.0, 7, {0, -10, 0, 0, 0, 0, 0}, 0);
    gesture(1.95, 7, {-18, -16, 12, -28, -12, -10, -20}, -4, 6);
    gesture(1.3, 7, {-30, -22, 24, -45, -25, -18, -35}, -8, 12);
    gesture(1.1, 7, {-27, -20, 16, -45, -10, -6, -20}, -5, 17);
    gesture(.2, 7, {-27, -20, 16, -45, -10, -6, -20}, -5, 17);
    gesture(1.6, 7, {-15, -12, 8, -22, -5, 0, -10}, 0, 5);
    gesture(1.65, 7, {0, -10, 0, 0, 0, 0, 0}, 0);
    gesture(1.0, 7, {}, 0);
  } else if (goal.name == "look_at") {
    const double angle = -std::remainder(goal.direction_deg.value_or(0.0), 360.0) * kDeg;
    yaw(angle, .75);
    yaw(angle, 1.5);
    yaw(y, .75);
  } else {
    throw RequestError(422, "unknown action: " + goal.name);
  }
  return steps;
}

void MotionEngine::submit(const ActionGoal &goal, double now) {
  if (goal.id.empty() || goal.id.size() > 128 || goal.name.empty() || goal.name.size() > 128 ||
      goal.timeout_ms < 1 || goal.timeout_ms > 30000 || !std::isfinite(now) ||
      !std::isfinite(goal.duration) || goal.duration <= 0 || goal.duration > 30 ||
      (goal.direction_deg && !std::isfinite(*goal.direction_deg)))
    throw RequestError(422, "invalid action id, timeout, duration, or direction");
  for (std::size_t i = 0; i < kJointCount; ++i)
    if (goal.targets[i] && (!std::isfinite(*goal.targets[i]) ||
        *goal.targets[i] < config_.lower[i] || *goal.targets[i] > config_.upper[i]))
      throw RequestError(422, "joint target exceeds configured limits");

  std::lock_guard<std::mutex> lock(mutex_);
  if (!fresh(now)) throw RequestError(503, "motor feedback is missing or stale");
  if (publishing_) sample(now);
  if (!plan_.empty() && !goal.preempt) throw RequestError(409, "another action is running");
  JointVector start = publishing_ ? command_.position : measured_;
  auto points = waypoints(goal, start);
  std::vector<Segment> plan;
  double total = 0;
  auto from = start;
  for (const auto &point : points) {
    auto to = from;
    double duration = point.duration;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (point.targets[i]) to[i] = *point.targets[i];
      // Peak derivative of the quintic position curve is 1.875.
      duration = std::max(duration, 1.875 * std::abs(to[i] - from[i]) / config_.max_velocity[i]);
    }
    plan.push_back({from, to, duration});
    total += duration;
    from = to;
  }
  if (total > goal.timeout_ms / 1000.0)
    throw RequestError(422, "timeout is shorter than the trajectory at configured velocity limits");
  command_.position = start;
  command_.velocity.fill(0);
  publishing_ = true;
  plan_ = std::move(plan);
  segment_ = 0;
  segment_start_ = now;
  deadline_ = now + goal.timeout_ms / 1000.0;
  action_id_ = goal.id;
  action_ = goal.name;
  state_ = "running";
}

void MotionEngine::sample(double now) {
  if (plan_.empty()) return;
  if (now > deadline_) {
    plan_.clear();
    command_.velocity.fill(0);
    state_ = "timed_out";
    return;
  }
  while (segment_ < plan_.size()) {
    const auto &s = plan_[segment_];
    if (now < segment_start_ + s.duration) break;
    command_.position = s.to;
    command_.velocity.fill(0);
    segment_start_ += s.duration;
    ++segment_;
  }
  if (segment_ == plan_.size()) {
    plan_.clear();
    state_ = "completed";
    return;
  }
  const auto &s = plan_[segment_];
  const double t = std::clamp((now - segment_start_) / s.duration, 0.0, 1.0);
  const double blend = t * t * t * (10 + t * (-15 + 6 * t));
  const double derivative = 30 * t * t * (1 - t) * (1 - t) / s.duration;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    command_.position[i] = s.from[i] + blend * (s.to[i] - s.from[i]);
    command_.velocity[i] = derivative * (s.to[i] - s.from[i]);
  }
}

std::optional<JointCommand> MotionEngine::tick(double now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!fresh(now)) {
    publishing_ = false;
    plan_.clear();
    command_.velocity.fill(0);
    state_ = "waiting_feedback";
    return std::nullopt;
  }
  if (!publishing_) return std::nullopt;
  sample(now);
  return command_;
}

void MotionEngine::stop(double now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (publishing_) sample(now);
  plan_.clear();
  command_.velocity.fill(0);
  state_ = "stopped";
}

MotionStatus MotionEngine::status(double now) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool ready = fresh(now);
  return {ready, ready && publishing_, feedback_time_ ? now - *feedback_time_ : -1.0,
          action_id_, action_, ready ? state_ : "waiting_feedback"};
}

}  // namespace chat2me_motion
