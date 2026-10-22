#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "chat2me_motion/motion_controller.hpp"

namespace chat2me_motion {

class UdsServer {
 public:
  UdsServer(std::string socket_path,
            std::shared_ptr<MotionController> controller);
  ~UdsServer();

  void start();
  void stop();

 private:
  void run();
  void handleClient(int client_fd);

  std::string socket_path_;
  std::shared_ptr<MotionController> controller_;
  std::atomic<bool> running_{false};
  int server_fd_{-1};
  std::thread thread_;
};

}  // namespace chat2me_motion
