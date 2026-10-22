#pragma once

#include <string>

namespace chat2me_motion {

struct ServerConfig {
  std::string socket_path = "/opt/chat2me/motion/motion.sock";
};

struct RosConfig {
  int domain_id = 0;
  bool localhost_only = false;
  std::string joint_state_topic = "motor/q2w_upper_motor_joint_state";
  std::string pt_control_topic = "motor/tt_upper_motor_pt_control_data";
  int control_hz = 50;
};

struct NeckConfig {
  double yaw_limit_deg = 70.0;     // Strictly ±70 deg
  double pitch_min_deg = -25.0;
  double pitch_max_deg = 25.0;
  double roll_min_deg = -20.0;
  double roll_max_deg = 20.0;
  double kp = 200.0;
  double kd = 5.0;
};

struct MotionConfig {
  ServerConfig server;
  RosConfig ros;
  NeckConfig neck;
};

MotionConfig loadConfig(const std::string &path);
std::string defaultConfigFileContent();

}  // namespace chat2me_motion
