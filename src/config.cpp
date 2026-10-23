#include "chat2me_motion/config.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace chat2me_motion {
namespace {

template <typename T>
void scalar(const YAML::Node &node, const char *key, T &value) {
  if (node[key]) value = node[key].as<T>();
}

void vector(const YAML::Node &node, const char *key, JointVector &values) {
  if (!node[key]) return;
  auto input = node[key].as<std::vector<double>>();
  if (input.size() != kJointCount) throw std::runtime_error(std::string(key) + " must contain 17 values");
  std::copy(input.begin(), input.end(), values.begin());
}

void targets(const YAML::Node &node, const char *key, std::size_t offset, std::size_t count,
             JointTargets &values) {
  if (!node[key]) return;
  auto input = node[key].as<std::vector<double>>();
  if (input.size() != count) throw std::runtime_error(std::string(key) + " has wrong joint count");
  for (std::size_t i = 0; i < count; ++i) {
    if (!std::isfinite(input[i])) throw std::runtime_error("non-finite joint target");
    values[offset + i] = input[i];
  }
}

}  // namespace

void validateConfig(const MotionConfig &c) {
  if (c.domain_id < 0 || c.domain_id > 232 || c.control_hz < 20 || c.control_hz > 200 ||
      !std::isfinite(c.feedback_timeout) || c.feedback_timeout <= 0.0 || c.feedback_timeout > 5.0)
    throw std::runtime_error("invalid ROS domain, control rate, or feedback timeout");
  if (c.socket_path.empty() || c.socket_path.front() != '/' || c.socket_path.size() >= 108)
    throw std::runtime_error("socket_path must be an absolute Unix socket path shorter than 108 bytes");
  if (c.command_topic.empty() || c.command_topic.front() != '/' ||
      c.command_topic.rfind("/motor/", 0) == 0 || c.command_topic == c.feedback_topic)
    throw std::runtime_error("joint_command_topic must be an absolute upper-command topic, outside /motor/");
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!std::isfinite(c.kp[i]) || c.kp[i] < 0 || !std::isfinite(c.kd[i]) || c.kd[i] < 0 ||
        !std::isfinite(c.lower[i]) || !std::isfinite(c.upper[i]) || c.lower[i] >= c.upper[i] ||
        !std::isfinite(c.max_velocity[i]) || c.max_velocity[i] <= 0)
      throw std::runtime_error("invalid joint gains, limits, or maximum velocity");
  }
  constexpr double yaw_limit = 70.0 * 3.14159265358979323846 / 180.0;
  if (c.lower[14] < -yaw_limit - 1e-9 || c.upper[14] > yaw_limit + 1e-9)
    throw std::runtime_error("head yaw limits must remain inside +/-70 degrees");
  for (const auto &[name, steps] : c.actions) {
    if (name.empty() || steps.empty() || steps.size() > 100)
      throw std::runtime_error("custom actions require a name and 1..100 waypoints");
    if (name == "nod" || name == "shake" || name == "look_around" || name == "think" ||
        name == "look_at" || name == "move_joints")
      throw std::runtime_error("custom action cannot replace a built-in action");
    for (const auto &step : steps) {
      if (!std::isfinite(step.duration) || step.duration <= 0 || step.duration > 30)
        throw std::runtime_error("waypoint duration must be in (0, 30] seconds");
      for (std::size_t i = 0; i < kJointCount; ++i)
        if (step.targets[i] && (!std::isfinite(*step.targets[i]) ||
            *step.targets[i] < c.lower[i] || *step.targets[i] > c.upper[i]))
          throw std::runtime_error("custom action target exceeds configured joint limits");
    }
  }
}

MotionConfig loadConfig(const std::string &path) {
  if (!std::ifstream(path)) throw std::runtime_error("cannot read motion config: " + path);
  const auto root = YAML::LoadFile(path);
  if (!root.IsMap()) throw std::runtime_error("motion configuration must be a mapping");
  MotionConfig c;
  auto ros = root["ros"];
  if (root["neck"] || (ros && ros["pt_control_topic"]))
    throw std::runtime_error("legacy direct-motor configuration: migrate using config.example.yml");
  auto server = root["server"];
  if (server) scalar(server, "socket_path", c.socket_path);
  if (ros) {
    scalar(ros, "domain_id", c.domain_id);
    scalar(ros, "localhost_only", c.localhost_only);
    scalar(ros, "joint_state_topic", c.feedback_topic);
    scalar(ros, "joint_command_topic", c.command_topic);
    scalar(ros, "control_hz", c.control_hz);
    scalar(ros, "feedback_timeout_seconds", c.feedback_timeout);
  }
  auto joints = root["joints"];
  if (joints) {
    vector(joints, "kp", c.kp);
    vector(joints, "kd", c.kd);
    vector(joints, "lower", c.lower);
    vector(joints, "upper", c.upper);
    vector(joints, "max_velocity", c.max_velocity);
  }
  auto actions = root["actions"];
  if (actions) {
    if (!actions.IsMap()) throw std::runtime_error("actions must be a mapping");
    for (const auto &entry : actions) {
      auto name = entry.first.as<std::string>();
      if (!entry.second.IsSequence()) throw std::runtime_error("action waypoints must be a sequence");
      auto &steps = c.actions[name];
      for (const auto &node : entry.second) {
        Waypoint step;
        scalar(node, "duration_seconds", step.duration);
        targets(node, "left_arm", 0, 7, step.targets);
        targets(node, "right_arm", 7, 7, step.targets);
        targets(node, "head", 14, 3, step.targets);
        steps.push_back(step);
      }
    }
  }
  validateConfig(c);
  return c;
}

std::string defaultConfigFileContent() {
  return R"(server:
  socket_path: /opt/chat2me/motion/motion.sock
ros:
  domain_id: 0
  localhost_only: false
  joint_state_topic: /motor/q2w_upper_motor_joint_state
  joint_command_topic: /throw/arm_joint_command
  control_hz: 50
  feedback_timeout_seconds: 0.5
actions: {}
)";
}

}  // namespace chat2me_motion
