#pragma once

#include <atomic>
#include <string>
#include <thread>

#include "chat2me_motion/motion_engine.hpp"

namespace chat2me_motion {

class UdsServer {
 public:
  UdsServer(std::string socket_path, MotionEngine &engine);
  ~UdsServer();
  void start();
  void stop();

 private:
  void run();
  void handleClient(int client_fd);
  std::string socket_path_;
  MotionEngine &engine_;
  std::atomic<bool> running_{false};
  int server_fd_{-1};
  std::thread thread_;
};

}  // namespace chat2me_motion
