#include <limits.h>
#include <unistd.h>

#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "chat2me_motion/config.hpp"
#include "chat2me_motion/motion_controller.hpp"
#include "chat2me_motion/service.hpp"
#include "chat2me_motion/uds_server.hpp"

namespace {

std::string executablePath() {
  std::vector<char> buffer(PATH_MAX + 1);
  const auto length = ::readlink("/proc/self/exe", buffer.data(), PATH_MAX);
  if (length < 0) {
    throw std::runtime_error("cannot resolve /proc/self/exe");
  }
  return std::string(buffer.data(), static_cast<std::size_t>(length));
}

void usage(std::ostream &stream) {
  stream << "Usage:\n"
         << "  chat2me-motion run [--config PATH]\n"
         << "  chat2me-motion service install [--force-config]\n"
         << "  chat2me-motion service uninstall\n"
         << "  chat2me-motion service status\n"
         << "  chat2me-motion --version\n";
}

}  // namespace

int main(int argc, char **argv) {
  try {
    std::vector<std::string> arguments;
    for (int index = 1; index < argc; ++index) {
      arguments.emplace_back(argv[index]);
    }
    if (!arguments.empty() && arguments.front() == "service") {
      return chat2me_motion::serviceCommand(
          std::vector<std::string>(arguments.begin() + 1, arguments.end()),
          executablePath());
    }
    if (!arguments.empty() && arguments.front() == "--version") {
      std::cout << CHAT2ME_MOTION_VERSION << '\n';
      return 0;
    }
    if (!arguments.empty() &&
        (arguments.front() == "--help" || arguments.front() == "-h")) {
      usage(std::cout);
      return 0;
    }

    std::string config_path = "/opt/chat2me/motion/config.yml";
    std::size_t index = 0;
    if (!arguments.empty() && arguments.front() == "run") {
      index = 1;
    }
    for (; index < arguments.size(); ++index) {
      if (arguments[index] == "--config" && index + 1 < arguments.size()) {
        config_path = arguments[++index];
      } else {
        throw std::runtime_error("unknown option: " + arguments[index]);
      }
    }

    const auto config = chat2me_motion::loadConfig(config_path);

    ::setenv("ROS_LOCALHOST_ONLY", config.ros.localhost_only ? "1" : "0", 1);
    const auto domain = std::to_string(config.ros.domain_id);
    ::setenv("ROS_DOMAIN_ID", domain.c_str(), 1);

    rclcpp::init(0, nullptr);
    auto controller = std::make_shared<chat2me_motion::MotionController>(config);

    chat2me_motion::UdsServer server(config.server.socket_path, controller);
    server.start();

    rclcpp::spin(controller);

    server.stop();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "chat2me-motion: " << error.what() << '\n';
    return 1;
  }
}
