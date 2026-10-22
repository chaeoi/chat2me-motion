#include "chat2me_motion/service.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "chat2me_motion/config.hpp"

namespace chat2me_motion {

namespace {

constexpr const char *kInstallDirectory = "/opt/chat2me/motion";
constexpr const char *kInstalledBinary = "/opt/chat2me/motion/chat2me-motion";
constexpr const char *kInstalledConfig = "/opt/chat2me/motion/config.yml";
constexpr const char *kUnitPath = "/etc/systemd/system/chat2me-motion.service";

void requireRoot() {
  if (::geteuid() != 0) {
    throw std::runtime_error("service changes require root; run with sudo");
  }
}

int runCommand(const std::vector<std::string> &arguments,
               bool allow_failure = false) {
  const auto child = ::fork();
  if (child < 0) {
    throw std::runtime_error("fork failed");
  }
  if (child == 0) {
    std::vector<char *> values;
    values.reserve(arguments.size() + 1);
    for (const auto &argument : arguments) {
      values.push_back(const_cast<char *>(argument.c_str()));
    }
    values.push_back(nullptr);
    ::execvp(values[0], values.data());
    _exit(127);
  }
  int status = 0;
  if (::waitpid(child, &status, 0) < 0) {
    throw std::runtime_error("waitpid failed");
  }
  const auto code = WIFEXITED(status) ? WEXITSTATUS(status) : 128;
  if (code != 0 && !allow_failure) {
    throw std::runtime_error(arguments.front() + " exited with status " +
                             std::to_string(code));
  }
  return code;
}

void writeFileAtomic(const std::string &path, const std::string &content,
                     mode_t mode) {
  const auto temporary = path + ".tmp." + std::to_string(::getpid());
  {
    std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
    if (!stream) {
      throw std::runtime_error("cannot write " + temporary);
    }
    stream << content;
    stream.flush();
    if (!stream) {
      throw std::runtime_error("cannot flush " + temporary);
    }
  }
  if (::chmod(temporary.c_str(), mode) != 0) {
    std::filesystem::remove(temporary);
    throw std::runtime_error("cannot set permissions on " + temporary);
  }
  std::filesystem::rename(temporary, path);
}

std::string systemdUnitContent() {
  return R"([Unit]
Description=Chat2Me Robot Motion Controller
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/opt/chat2me/motion
ExecStart=/opt/chat2me/motion/chat2me-motion run --config /opt/chat2me/motion/config.yml
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
)";
}

void installService(const std::vector<std::string> &arguments,
                    const std::string &executable_path) {
  requireRoot();
  bool force_config = false;
  for (const auto &arg : arguments) {
    if (arg == "--force-config") {
      force_config = true;
    } else {
      throw std::runtime_error("unknown install option: " + arg);
    }
  }

  std::filesystem::create_directories(kInstallDirectory);

  // Copy binary
  std::filesystem::copy_file(
      executable_path, kInstalledBinary,
      std::filesystem::copy_options::overwrite_existing);
  ::chmod(kInstalledBinary, 0755);

  // Write config if missing or forced
  if (force_config || !std::filesystem::exists(kInstalledConfig)) {
    writeFileAtomic(kInstalledConfig, defaultConfigFileContent(), 0644);
  }

  // Write systemd unit
  writeFileAtomic(kUnitPath, systemdUnitContent(), 0644);

  runCommand({"systemctl", "daemon-reload"});
  runCommand({"systemctl", "enable", "--now", "chat2me-motion.service"});

  std::cout << "已成功安装 chat2me-motion 服务并自启。\n";
}

void uninstallService(const std::vector<std::string> &arguments) {
  requireRoot();
  if (!arguments.empty()) {
    throw std::runtime_error("service uninstall takes no options");
  }

  runCommand({"systemctl", "disable", "--now", "chat2me-motion.service"}, true);
  std::filesystem::remove(kUnitPath);
  runCommand({"systemctl", "daemon-reload"});
  runCommand({"systemctl", "reset-failed", "chat2me-motion.service"}, true);

  // Clean directory including socket and configs
  std::filesystem::remove_all(kInstallDirectory);

  std::cout << "已彻底卸载 chat2me-motion 服务并删除全部程序、配置与文件。\n";
}

}  // namespace

int serviceCommand(const std::vector<std::string> &arguments,
                   const std::string &executable_path) {
  if (arguments.empty()) {
    throw std::runtime_error("service requires install, uninstall or status");
  }
  const std::vector<std::string> options(arguments.begin() + 1,
                                         arguments.end());
  if (arguments.front() == "install") {
    installService(options, executable_path);
    return 0;
  }
  if (arguments.front() == "uninstall") {
    uninstallService(options);
    return 0;
  }
  if (arguments.front() == "status") {
    if (!options.empty()) {
      throw std::runtime_error("service status takes no options");
    }
    return runCommand({"systemctl", "status", "chat2me-motion.service"}, true);
  }
  throw std::runtime_error("unknown service command: " + arguments.front());
}

}  // namespace chat2me_motion
