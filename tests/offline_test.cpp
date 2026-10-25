#include "chat2me_motion/motion_engine.hpp"
#include "chat2me_motion/protocol.hpp"
#include "chat2me_motion/uds_server.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

#include <picojson.h>

using namespace chat2me_motion;
namespace {
int checks = 0;
void check(bool condition, const std::string &message) {
  ++checks;
  if (!condition) throw std::runtime_error(message);
}
void near(double a, double b, const std::string &message) { check(std::abs(a - b) < 1e-8, message); }

template <typename F> void rejected(F function, int code) {
  try { function(); } catch (const RequestError &e) {
    check(e.status == code, "unexpected rejection status");
    return;
  }
  throw std::runtime_error("request was accepted unexpectedly");
}

std::vector<double> feedback() {
  std::vector<double> values(32, 0);
  for (std::size_t i = 3; i < 20; ++i) values[i] = (i - 3) * .01;
  return values;
}

double measuredJoint(const std::vector<double> &values, std::size_t joint) {
  if (joint < 14) return values[joint + 3];
  if (joint == kHeadYaw) return values[19];
  if (joint == kHeadPitch) return values[18];
  if (joint == kHeadRoll) return values[17];
  throw std::out_of_range("invalid joint index");
}

ActionGoal move() {
  ActionGoal goal;
  goal.id = "move-1";
  goal.name = "move_joints";
  goal.targets[0] = .3;
  goal.targets[7] = -.2;
  goal.targets[14] = -.3;
  goal.duration = 2;
  return goal;
}

void testTrajectoryAndPacking() {
  MotionConfig config;
  MotionEngine engine(config);
  check(!engine.tick(0), "startup must not produce commands");
  rejected([&] { engine.submit(move(), 0); }, 503);
  auto measured = feedback();
  engine.updateFeedback(measured, 0);
  check(!engine.tick(0), "feedback alone must not produce commands");
  check(engine.status(0).state == "waiting_action", "fresh idle feedback reported as missing");
  engine.submit(move(), 0);
  const auto initial = *engine.tick(0);
  near(initial.position[kHeadYaw], measured[19], "yaw feedback must use CAN ID 20");
  near(initial.position[kHeadPitch], measured[18], "pitch feedback must use CAN ID 19");
  near(initial.position[kHeadRoll], measured[17], "roll feedback must use CAN ID 18");
  for (int frame = 0; frame <= 150; ++frame) {
    const double now = frame * .02;
    engine.updateFeedback(measured, now);
    auto command = engine.tick(now);
    check(command.has_value(), "commands must continue after completing a trajectory");
    for (std::size_t i = 0; i < 17; ++i)
      check(std::abs(command->velocity[i]) <= config.max_velocity[i] + 1e-8, "velocity limit exceeded");
    for (auto i : {1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 15, 16})
      near(command->position[i], measuredJoint(measured, i), "uncommanded joint changed");
  }
  auto command = *engine.tick(3);
  near(command.position[0], .3, "left arm target missing");
  near(command.position[7], -.2, "right arm target missing");
  near(command.position[14], -.3, "head target missing");
  auto packed = packCommand(command, config);
  check(packed.position.size() == 34 && packed.velocity.size() == 34 && packed.effort.size() == 17,
        "incorrect _head.py command shape");
  for (std::size_t i = 0; i < 17; ++i) {
    near(packed.position[i], command.position[i], "q packed into wrong offset");
    near(packed.position[17 + i], config.kp[i], "kp packed into wrong offset");
    near(packed.velocity[17 + i], config.kd[i], "kd packed into wrong offset");
    near(packed.effort[i], 0, "unexpected feed-forward torque");
  }
  check(packed.names[0] == "19_left_shoulder_pitch" && packed.names[7] == "26_right_shoulder_pitch" &&
        packed.names[14] == "16_neck_yaw" && packed.names[15] == "17_neck_pitch" &&
        packed.names[16] == "18_neck_roll", "joint names do not match the URDF");
  check(engine.status(3).state == "completed", "missing completion status");
}

void testHeadPreservesBothArms() {
  for (const auto &name : {"nod", "think"}) {
    MotionConfig config;
    config.arm_gestures = false;
    MotionEngine engine(config);
    auto measured = feedback();
    engine.updateFeedback(measured, 0);
    ActionGoal goal;
    goal.id = name;
    goal.name = name;
    engine.submit(goal, 0);
    for (int i = 0; i <= 200; ++i) {
      const auto now = i * .02;
      engine.updateFeedback(measured, now);
      auto command = *engine.tick(now);
      for (std::size_t j = 0; j < 14; ++j)
        near(command.position[j], measuredJoint(measured, j), "head-only action moved an arm");
      const auto packed = packCommand(command, config);
      for (std::size_t j = 0; j < 14; ++j)
        check(packed.position[17 + j] > 0, "head-only action cleared arm gains");
    }
    check(engine.status(4).state == "completed", "head-only action retained arm staging delays");
    for (std::size_t j = kHeadYaw; j < kJointCount; ++j)
      near(engine.tick(4)->position[j], measuredJoint(measured, j), "head-only action did not restore its starting pose");
  }
}

void testYawActionsDoNotRecenterOtherAxes() {
  for (const auto &name : {"shake", "look_around", "look_at"}) {
    MotionConfig config;
    MotionEngine engine(config);
    auto measured = feedback();
    measured[19] = .25;
    measured[18] = .20;
    measured[17] = -.12;
    engine.updateFeedback(measured, 0);
    ActionGoal goal;
    goal.id = std::string("yaw-") + name;
    goal.name = name;
    goal.direction_deg = 90;
    engine.submit(goal, 0);
    double minimum_yaw = measuredJoint(measured, kHeadYaw);
    for (int frame = 0; frame <= 500; ++frame) {
      const double now = frame * .02;
      engine.updateFeedback(measured, now);
      auto command = engine.tick(now);
      check(command.has_value(), "yaw action stopped publishing");
      minimum_yaw = std::min(minimum_yaw, command->position[kHeadYaw]);
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        if (joint == kHeadYaw) continue;
        near(command->position[joint], measuredJoint(measured, joint), "yaw action changed an unrelated joint");
        near(command->velocity[joint], 0, "yaw action commanded pitch/roll or arm velocity");
      }
    }
    check(minimum_yaw < measuredJoint(measured, kHeadYaw) - .1, "yaw action never turned the head");
    near(engine.tick(10)->position[kHeadYaw], measuredJoint(measured, kHeadYaw), "yaw action did not return to initial yaw");
    check(engine.status(10).state == "completed", "yaw action exceeded its timeout");
  }
}

void testArmGesturesReturnToTheirInitialPose() {
  for (const auto &name : {"nod", "think"}) {
    MotionConfig config;
    MotionEngine engine(config);
    auto measured = feedback();
    measured[19] = .25;
    measured[18] = .15;
    measured[17] = -.12;
    engine.updateFeedback(measured, 0);
    ActionGoal goal;
    goal.id = std::string("gesture-") + name;
    goal.name = name;
    engine.submit(goal, 0);
    const bool left = std::string(name) == "nod";
    const std::size_t offset = left ? 0 : 7;
    MotionEngine sync_engine(config);
    sync_engine.updateFeedback(measured, 0);
    sync_engine.submit(goal, 0);
    const double open_duration = std::max(
        .8, 1.875 * 8 * 3.14159265358979323846 / 180.0 /
                config.max_velocity[offset + 1]);
    const double arm_delta_degrees = left ? 28.0 : 25.0;
    const double gesture_delta_degrees = left ? -14.0 : -16.0;
    const double head_delta_degrees = left ? 8.0 : -8.0;
    const double synchronized_duration = std::max(
        left ? 1.3 : 1.4,
        1.875 * arm_delta_degrees * 3.14159265358979323846 / 180.0 /
            config.max_velocity[offset + 3]);
    const double synchronized_midpoint = open_duration + synchronized_duration / 2.0;
    for (double now = 0; now < synchronized_midpoint; now += .02) {
      sync_engine.updateFeedback(measured, now);
      (void)sync_engine.tick(now);
    }
    sync_engine.updateFeedback(measured, synchronized_midpoint);
    const auto synchronized = *sync_engine.tick(synchronized_midpoint);
    const double arm_progress =
        (synchronized.position[offset] - measured[offset + 3]) /
        (gesture_delta_degrees * 3.14159265358979323846 / 180.0);
    const double head_progress =
        (synchronized.position[kHeadPitch] - measuredJoint(measured, kHeadPitch)) /
        (head_delta_degrees * 3.14159265358979323846 / 180.0);
    check(std::abs(arm_progress - head_progress) < 1e-5 &&
              arm_progress > .05 && arm_progress < .95,
          "head and arm progress differs: arm=" + std::to_string(arm_progress) +
              " head=" + std::to_string(head_progress));

    std::array<double, 7> excursion{};
    for (int frame = 0; frame <= 500; ++frame) {
      const double now = frame * .02;
      engine.updateFeedback(measured, now);
      auto command = *engine.tick(now);
      for (std::size_t i = 0; i < excursion.size(); ++i)
        excursion[i] = std::max(excursion[i], std::abs(command.position[offset + i] - measured[offset + i + 3]));
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        check(command.position[joint] >= config.lower[joint] - 1e-8 &&
              command.position[joint] <= config.upper[joint] + 1e-8, "gesture exceeded a joint limit");
        check(std::abs(command.velocity[joint]) <= config.max_velocity[joint] + 1e-8,
              "gesture exceeded a joint velocity limit");
        const bool selected = (joint >= offset && joint < offset + 7) || joint == kHeadPitch ||
                              (!left && joint == kHeadRoll);
        if (!selected) near(command.position[joint], measuredJoint(measured, joint), "gesture changed an unrelated joint");
      }
    }
    for (const double delta : excursion)
      check(delta > .1, "gesture did not coordinate all seven arm joints");
    const auto final = *engine.tick(10);
    for (std::size_t joint = 0; joint < kJointCount; ++joint)
      near(final.position[joint], measuredJoint(measured, joint), "gesture did not return to its initial pose");
    check(engine.status(10).state == "completed", "arm gesture exceeded its timeout");
  }
}

void testFeedbackLossAndPreemption() {
  MotionEngine engine(MotionConfig{});
  auto measured = feedback();
  engine.updateFeedback(measured, 0);
  engine.submit(move(), 0);
  auto before = *engine.tick(.2);
  auto replacement = move();
  replacement.id = "move-2";
  replacement.preempt = false;
  rejected([&] { engine.submit(replacement, .2); }, 409);
  replacement.preempt = true;
  replacement.targets[0] = -.1;
  engine.submit(replacement, .2);
  near(engine.tick(.2)->position[0], before.position[0], "preemption jumped in position");
  auto bad = measured;
  bad[17] = std::numeric_limits<double>::quiet_NaN();
  engine.updateFeedback(bad, .4);
  check(!engine.tick(.6), "invalid feedback refreshed the watchdog");
  engine.updateFeedback(measured, .61);
  check(!engine.tick(.61), "reconnect resumed an obsolete action");
  check(engine.status(.61).state == "waiting_action", "reconnect still reported a running action");
  engine.submit(move(), .61);
  engine.updateFeedback(measured, 1.3);
  check(!engine.tick(1.3), "late feedback resumed motion before a watchdog tick");
  check(engine.status(1.3).state == "waiting_action", "late feedback retained obsolete action status");
  engine.submit(move(), 1.3);
  engine.stop(1.4);
  check(engine.status(1.4).state == "stopped", "stop did not cancel the action");
  const auto stopped = engine.tick(1.4);
  check(stopped.has_value(), "stop should hold the current pose");
  for (const auto velocity : stopped->velocity) near(velocity, 0, "stop left a nonzero velocity");
}

void testDeadlineAndValidation() {
  MotionEngine engine(MotionConfig{});
  engine.updateFeedback(std::vector<double>(32, 0), 0);
  auto goal = move();
  goal.targets = {};
  goal.targets[14] = .01;
  goal.duration = .05;
  goal.timeout_ms = 100;
  engine.submit(goal, 0);
  const auto before = *engine.tick(.02);
  const auto after = *engine.tick(.11);
  near(after.position[14], before.position[14], "expired action jumped to its future target");
  check(engine.status(.11).state == "timed_out", "deadline not enforced");
  goal.name = "unknown";
  rejected([&] { engine.submit(goal, .11); }, 422);
  goal = move();
  goal.targets[0] = 50;
  rejected([&] { engine.submit(goal, .11); }, 422);
  goal = move();
  goal.timeout_ms = 10;
  rejected([&] { engine.submit(goal, .11); }, 422);
  goal = move();
  goal.targets[0] = std::numeric_limits<double>::infinity();
  rejected([&] { engine.submit(goal, .11); }, 422);
}

void testProtocol() {
  MotionEngine engine(MotionConfig{});
  check(handleRequest("GET", "/health", "", engine, 0).status == 503, "health claimed readiness without feedback");
  const std::string nod = R"({"id":"n1","name":"nod","params":{},"preempt":true,"timeout_ms":10000})";
  check(handleRequest("POST", "/action", nod, engine, 0).status == 503, "action accepted without feedback");
  engine.updateFeedback(feedback(), 0);
  check(handleRequest("GET", "/health", "", engine, 0).status == 200, "fresh feedback not ready");
  check(handleRequest("GET", "/action", "", engine, 0).status == 405, "wrong method accepted");
  check(handleRequest("POST", "/bogus", nod, engine, 0).status == 404, "wrong path accepted");
  check(handleRequest("POST", "/action", "{", engine, 0).status == 400, "malformed JSON accepted");
  check(handleRequest("POST", "/action", nod + " false", engine, 0).status == 400, "trailing JSON accepted");
  for (const auto &body : {
       R"({"id":"x","name":"wave"})",
       R"({"id":"x","name":"nod","preempt":"false"})",
       R"({"id":"x","name":"nod","timeout_ms":1.5})",
       R"({"id":"x","name":"nod","params":{"head":[0,0,0]}})",
       R"({"id":"x","name":"move_joints","params":{"left_arm":[0,0]}})",
       R"({"id":"x","name":"look_at","params":{"direction_deg":"90"}})"})
    check(handleRequest("POST", "/action", body, engine, 0).status == 422, "invalid action request accepted");
  const auto parsed = parseAction(R"({"id":"l1","name":"look_at","params":{"direction_deg":90,"fallback":"speaker"}})");
  near(*parsed.direction_deg, 90, "nested sound direction lost");
  const auto response = handleRequest("POST", "/action", nod, engine, 0);
  check(response.status == 202, "valid request rejected");
  picojson::value result;
  check(picojson::parse(result, response.body).empty(), "response is not JSON");
  check(result.get("ok").get<bool>() && result.get("id").get<std::string>() == "n1" &&
        result.get("status").get<std::string>() == "accepted", "missing acceptance envelope");
  check(handleRequest("POST", "/stop", "", engine, 0).status == 200, "stop endpoint failed");
}

void testConfig() {
  const auto path = std::filesystem::temp_directory_path() / ("motion-config-test-" + std::to_string(::getpid()));
  { std::ofstream file(path); file << defaultConfigFileContent(); }
  auto config = loadConfig(path.string());
  check(config.command_topic == "/throw/arm_joint_command", "wrong default command topic");
  check(config.arm_gestures, "default configuration disabled the requested arm gestures");
  { std::ofstream file(path); file << "arm_gestures: false\n"; }
  check(!loadConfig(path.string()).arm_gestures, "head-only configuration was not respected");
  { std::ofstream file(path); file << "ros:\n  pt_control_topic: /motor/tt_upper_motor_pt_control_data\n"; }
  bool failed = false;
  try { loadConfig(path.string()); } catch (const std::exception &) { failed = true; }
  check(failed, "legacy direct-motor config accepted");
  std::filesystem::remove(path);
  failed = false;
  try { loadConfig(path.string()); } catch (const std::exception &) { failed = true; }
  check(failed, "missing config silently used defaults");
}

std::string http(const std::string &path, const std::string &request) {
  const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
  check(fd >= 0, "test socket failed");
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
  check(::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0, "test connect failed");
  check(::send(fd, request.data(), request.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(request.size()), "test send failed");
  ::shutdown(fd, SHUT_WR);
  std::string response;
  char buffer[1024];
  ssize_t count;
  while ((count = ::recv(fd, buffer, sizeof(buffer), 0)) > 0) response.append(buffer, count);
  ::close(fd);
  return response;
}

void testOfflineSocket() {
  MotionEngine engine(MotionConfig{});
  const auto path = (std::filesystem::temp_directory_path() / ("motion-uds-test-" + std::to_string(::getpid()) + ".sock")).string();
  UdsServer server(path, engine);
  server.start();
  check(http(path, "GET /health HTTP/1.1\r\nHost: test\r\n\r\n").find("503") != std::string::npos, "UDS health failed");
  check(http(path, "POST /action HTTP/1.1\r\ncontent-length: nope\r\n\r\n").find("400") != std::string::npos,
        "bad Content-Length crashed the server");
  check(http(path, "POST /action HTTP/1.1\r\ncontent-length: 999999\r\n\r\n").find("400") != std::string::npos,
        "oversized body accepted");
  UdsServer duplicate(path, engine);
  bool failed = false;
  try { duplicate.start(); } catch (const std::exception &) { failed = true; }
  check(failed && std::filesystem::exists(path), "second server replaced a live socket");
  check(http(path, "GET /health HTTP/1.1\r\nHost: test\r\n\r\n").find("503") != std::string::npos, "server died after malformed request");
  server.stop();
  check(!std::filesystem::exists(path), "socket not cleaned up");
}
}  // namespace

int main() {
  try {
    testTrajectoryAndPacking();
    testHeadPreservesBothArms();
    testYawActionsDoNotRecenterOtherAxes();
    testArmGesturesReturnToTheirInitialPose();
    testFeedbackLossAndPreemption();
    testDeadlineAndValidation();
    testProtocol();
    testConfig();
    testOfflineSocket();
    std::cout << checks << " offline checks passed; no ROS runtime or motor access\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "offline check failed: " << e.what() << '\n';
    return 1;
  }
}
