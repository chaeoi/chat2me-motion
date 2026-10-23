#include "chat2me_motion/protocol.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <set>

#include <picojson.h>

namespace chat2me_motion {
namespace {
using Json = picojson::value;
using Object = picojson::object;

const Json *field(const Object &object, const std::string &key) {
  auto it = object.find(key);
  return it == object.end() ? nullptr : &it->second;
}

void checkKeys(const Object &object, const std::set<std::string> &allowed) {
  for (const auto &[key, value] : object) {
    (void)value;
    if (allowed.count(key) == 0) throw RequestError(422, "unknown field: " + key);
  }
}

std::string string(const Json *value, const char *name) {
  if (!value || !value->is<std::string>()) throw RequestError(422, std::string(name) + " must be a string");
  return value->get<std::string>();
}

double number(const Json *value, const char *name) {
  if (!value || !value->is<double>() || !std::isfinite(value->get<double>()))
    throw RequestError(422, std::string(name) + " must be a finite number");
  return value->get<double>();
}

int integer(const Json *value, const char *name) {
  const double result = number(value, name);
  if (result < 1 || result > 30000 || std::floor(result) != result)
    throw RequestError(422, std::string(name) + " must be an integer in 1..30000");
  return static_cast<int>(result);
}

void targets(const Object &params, const char *key, std::size_t offset, std::size_t count,
             JointTargets &values) {
  auto value = field(params, key);
  if (!value) return;
  if (!value->is<picojson::array>() || value->get<picojson::array>().size() != count)
    throw RequestError(422, std::string(key) + " has wrong joint count");
  const auto &input = value->get<picojson::array>();
  for (std::size_t i = 0; i < count; ++i) values[offset + i] = number(&input[i], key);
}

HttpResponse error(int code, const std::string &message) {
  return {code, Json(Object{{"ok", Json(false)}, {"error", Json(message)}}).serialize()};
}
}  // namespace

ActionGoal parseAction(const std::string &body) {
  if (body.empty() || body.size() > 16384) throw RequestError(400, "empty or oversized JSON body");
  Json root;
  std::string parse_error;
  auto end = picojson::parse(root, body.begin(), body.end(), &parse_error);
  if (!parse_error.empty() || !root.is<Object>()) throw RequestError(400, "invalid JSON object");
  for (; end != body.end(); ++end)
    if (!std::isspace(static_cast<unsigned char>(*end))) throw RequestError(400, "trailing JSON data");
  const auto &object = root.get<Object>();
  checkKeys(object, {"id", "name", "params", "preempt", "timeout_ms", "direction_deg"});
  ActionGoal goal;
  goal.id = string(field(object, "id"), "id");
  goal.name = string(field(object, "name"), "name");
  if (auto value = field(object, "preempt")) {
    if (!value->is<bool>()) throw RequestError(422, "preempt must be boolean");
    goal.preempt = value->get<bool>();
  }
  if (auto value = field(object, "timeout_ms")) goal.timeout_ms = integer(value, "timeout_ms");
  if (auto value = field(object, "direction_deg")) goal.direction_deg = number(value, "direction_deg");
  if (auto params_value = field(object, "params")) {
    if (!params_value->is<Object>()) throw RequestError(422, "params must be an object");
    const auto &params = params_value->get<Object>();
    checkKeys(params, {"left_arm", "right_arm", "head", "duration_ms", "direction_deg", "fallback"});
    if (auto value = field(params, "direction_deg")) {
      if (goal.direction_deg) throw RequestError(422, "direction_deg must appear only once");
      goal.direction_deg = number(value, "direction_deg");
    }
    if (auto value = field(params, "duration_ms")) goal.duration = integer(value, "duration_ms") / 1000.0;
    if (auto value = field(params, "fallback")) {
      auto fallback = string(value, "fallback");
      if (fallback != "speaker" && fallback != "front") throw RequestError(422, "invalid direction fallback");
    }
    targets(params, "left_arm", 0, 7, goal.targets);
    targets(params, "right_arm", 7, 7, goal.targets);
    targets(params, "head", 14, 3, goal.targets);
  }
  if (goal.name != "move_joints" &&
      std::any_of(goal.targets.begin(), goal.targets.end(), [](const auto &v) { return v.has_value(); }))
    throw RequestError(422, "joint targets are only accepted by move_joints");
  if (goal.name != "look_at" && goal.direction_deg)
    throw RequestError(422, "direction_deg is only accepted by look_at");
  return goal;
}

HttpResponse handleRequest(const std::string &method, const std::string &path,
                          const std::string &body, MotionEngine &engine, double now) {
  try {
    if (path == "/health") {
      if (method != "GET") return error(405, "use GET /health");
      auto status = engine.status(now);
      picojson::array actions;
      for (const auto &name : engine.supportedActions()) actions.emplace_back(name);
      Object result{{"ready", Json(status.ready)}, {"publishing", Json(status.publishing)},
                    {"state", Json(status.state)}, {"feedback_age_ms", Json(status.feedback_age * 1000)},
                    {"id", Json(status.action_id)}, {"action", Json(status.action)}, {"actions", Json(actions)}};
      return {status.ready ? 200 : 503, Json(result).serialize()};
    }
    if (path != "/action" && path != "/stop") return error(404, "not found");
    if (method != "POST") return error(405, "use POST for motion requests");
    if (path == "/stop") {
      engine.stop(now);
      return {200, "{\"ok\":true,\"status\":\"stopped\"}"};
    }
    auto goal = parseAction(body);
    engine.submit(goal, now);
    Object result{{"ok", Json(true)}, {"status", Json(std::string("accepted"))},
                  {"id", Json(goal.id)}, {"action", Json(goal.name)}};
    return {202, Json(result).serialize()};
  } catch (const RequestError &e) {
    return error(e.status, e.what());
  }
}

}  // namespace chat2me_motion
