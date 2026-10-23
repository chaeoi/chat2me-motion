#include "chat2me_motion/uds_server.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  auto now = [] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  };
  chat2me_motion::MotionEngine engine(chat2me_motion::MotionConfig{});
  engine.updateFeedback(std::vector<double>(32, 0), now());
  chat2me_motion::UdsServer server(argv[1], engine);
  server.start();
  std::atomic<bool> running{true};
  std::thread feedback([&] {
    while (running) {
      engine.updateFeedback(std::vector<double>(32, 0), now());
      (void)engine.tick(now());
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });
  std::cout << "ready (memory only; no ROS)" << std::endl;
  std::string line;
  std::getline(std::cin, line);
  server.stop();
  running = false;
  feedback.join();
}
