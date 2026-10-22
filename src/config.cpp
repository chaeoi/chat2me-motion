#include "chat2me_motion/config.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace chat2me_motion {

namespace {

std::string trim(const std::string &str) {
  const auto start = str.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) return "";
  const auto end = str.find_last_not_of(" \t\r\n");
  return str.substr(start, end - start + 1);
}

}  // namespace

std::string defaultConfigFileContent() {
  return R"(# chat2me-motion default configuration
server:
  socket_path: "/opt/chat2me/motion/motion.sock"

ros:
  domain_id: 0
  localhost_only: false
  joint_state_topic: "motor/q2w_upper_motor_joint_state"
  pt_control_topic: "motor/tt_upper_motor_pt_control_data"
  control_hz: 50

neck:
  yaw_limit_deg: 70.0
  pitch_min_deg: -25.0
  pitch_max_deg: 25.0
  roll_min_deg: -20.0
  roll_max_deg: 20.0
  kp: 200.0
  kd: 5.0
)";
}

MotionConfig loadConfig(const std::string &path) {
  MotionConfig config;
  std::ifstream file(path);
  if (!file.is_open()) {
    // If not found, return defaults
    return config;
  }

  std::string line;
  std::string section;
  while (std::getline(file, line)) {
    const auto comment_pos = line.find('#');
    if (comment_pos != std::string::npos) {
      line = line.substr(0, comment_pos);
    }
    line = trim(line);
    if (line.empty()) continue;

    if (line.back() == ':') {
      section = line.substr(0, line.size() - 1);
      continue;
    }

    const auto sep = line.find(':');
    if (sep == std::string::npos) continue;

    std::string key = trim(line.substr(0, sep));
    std::string value = trim(line.substr(sep + 1));
    if (value.front() == '"' && value.back() == '"' && value.size() >= 2) {
      value = value.substr(1, value.size() - 2);
    }

    try {
      if (section == "server") {
        if (key == "socket_path") config.server.socket_path = value;
      } else if (section == "ros") {
        if (key == "domain_id") config.ros.domain_id = std::stoi(value);
        else if (key == "localhost_only") config.ros.localhost_only = (value == "true" || value == "1");
        else if (key == "joint_state_topic") config.ros.joint_state_topic = value;
        else if (key == "pt_control_topic") config.ros.pt_control_topic = value;
        else if (key == "control_hz") config.ros.control_hz = std::stoi(value);
      } else if (section == "neck") {
        if (key == "yaw_limit_deg") config.neck.yaw_limit_deg = std::stod(value);
        else if (key == "pitch_min_deg") config.neck.pitch_min_deg = std::stod(value);
        else if (key == "pitch_max_deg") config.neck.pitch_max_deg = std::stod(value);
        else if (key == "roll_min_deg") config.neck.roll_min_deg = std::stod(value);
        else if (key == "roll_max_deg") config.neck.roll_max_deg = std::stod(value);
        else if (key == "kp") config.neck.kp = std::stod(value);
        else if (key == "kd") config.neck.kd = std::stod(value);
      }
    } catch (...) {
      // Ignore invalid scalar parse errors and keep default
    }
  }

  return config;
}

}  // namespace chat2me_motion
