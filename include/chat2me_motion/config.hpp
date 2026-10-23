#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace chat2me_motion {

constexpr std::size_t kJointCount = 17;
using JointVector = std::array<double, kJointCount>;
using JointTargets = std::array<std::optional<double>, kJointCount>;

struct Waypoint {
  JointTargets targets{};
  double duration = 1.0;
};

struct MotionConfig {
  std::string socket_path = "/opt/chat2me/motion/motion.sock";
  int domain_id = 0;
  bool localhost_only = false;
  std::string feedback_topic = "/motor/q2w_upper_motor_joint_state";
  std::string command_topic = "/throw/arm_joint_command";
  int control_hz = 50;
  double feedback_timeout = 0.5;
  JointVector kp{100, 80, 60, 60, 40, 40, 40, 100, 80, 60, 60, 40, 40, 40, 40, 40, 40};
  JointVector kd{5, 4, 3, 3, 2, 2, 2, 5, 4, 3, 3, 2, 2, 2, 2, 2, 2};
  JointVector lower{-3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159,
                    -3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159,
                    -1.2217304764, -0.4363323130, -0.3490658504};
  JointVector upper{3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159,
                    3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159,
                    1.2217304764, 0.4363323130, 0.3490658504};
  JointVector max_velocity{0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
                          0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1.2, 1.2, 1.2};
  std::map<std::string, std::vector<Waypoint>> actions;
};

void validateConfig(const MotionConfig &config);
MotionConfig loadConfig(const std::string &path);
std::string defaultConfigFileContent();

}  // namespace chat2me_motion
