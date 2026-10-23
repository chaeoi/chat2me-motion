#pragma once

#include "chat2me_motion/motion_engine.hpp"

namespace chat2me_motion {
struct HttpResponse {
  int status;
  std::string body;
};

ActionGoal parseAction(const std::string &body);
HttpResponse handleRequest(const std::string &method, const std::string &path,
                          const std::string &body, MotionEngine &engine, double now);
}  // namespace chat2me_motion
