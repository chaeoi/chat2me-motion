#pragma once

#include <mutex>
#include <stdexcept>

#include "chat2me_motion/config.hpp"

namespace chat2me_motion {

class RequestError : public std::runtime_error {
 public:
  RequestError(int code, const std::string &message) : std::runtime_error(message), status(code) {}
  int status;
};

struct ActionGoal {
  std::string id;
  std::string name;
  bool preempt = true;
  int timeout_ms = 10000;
  double duration = 1.0;
  std::optional<double> direction_deg;
  JointTargets targets{};
};

struct JointCommand {
  JointVector position{};
  JointVector velocity{};
};

struct PackedCommand {
  std::array<std::string, kJointCount> names;
  std::array<double, 2 * kJointCount> position{};
  std::array<double, 2 * kJointCount> velocity{};
  JointVector effort{};
};

struct MotionStatus {
  bool ready = false;
  bool publishing = false;
  double feedback_age = -1.0;
  std::string action_id;
  std::string action;
  std::string state = "waiting_feedback";
};

PackedCommand packCommand(const JointCommand &command, const MotionConfig &config);

// Time and feedback are supplied by the caller; no ROS or actuator access.
class MotionEngine {
 public:
  explicit MotionEngine(MotionConfig config);
  void updateFeedback(const std::vector<double> &positions, double now);
  void submit(const ActionGoal &goal, double now);
  std::optional<JointCommand> tick(double now);
  void stop(double now);
  MotionStatus status(double now) const;
  std::vector<std::string> supportedActions() const;

 private:
  struct Segment {
    JointVector from;
    JointVector to;
    double duration;
  };
  bool fresh(double now) const;
  void sample(double now);
  std::vector<Waypoint> waypoints(const ActionGoal &goal, const JointVector &start) const;

  MotionConfig config_;
  mutable std::mutex mutex_;
  std::optional<double> feedback_time_;
  JointVector measured_{};
  JointCommand command_;
  bool publishing_ = false;
  std::vector<Segment> plan_;
  std::size_t segment_ = 0;
  double segment_start_ = 0.0;
  double deadline_ = 0.0;
  std::string action_id_;
  std::string action_;
  std::string state_ = "waiting_feedback";
};

}  // namespace chat2me_motion
