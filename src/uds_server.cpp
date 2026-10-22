#include "chat2me_motion/uds_server.hpp"

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <sstream>

namespace chat2me_motion {

namespace {

bool parseJsonValue(const std::string &json, const std::string &key,
                    std::string &out_val) {
  const std::string pattern = "\"" + key + "\"";
  auto pos = json.find(pattern);
  if (pos == std::string::npos) return false;
  pos = json.find(':', pos + pattern.size());
  if (pos == std::string::npos) return false;
  pos = json.find_first_not_of(" \t\r\n", pos + 1);
  if (pos == std::string::npos) return false;

  if (json[pos] == '"') {
    const auto end = json.find('"', pos + 1);
    if (end == std::string::npos) return false;
    out_val = json.substr(pos + 1, end - pos - 1);
    return true;
  } else {
    const auto end = json.find_first_of(",}\r\n ", pos);
    out_val = json.substr(pos, (end == std::string::npos) ? std::string::npos : end - pos);
    return true;
  }
}

}  // namespace

UdsServer::UdsServer(std::string socket_path,
                     std::shared_ptr<MotionController> controller)
    : socket_path_(std::move(socket_path)), controller_(std::move(controller)) {}

UdsServer::~UdsServer() {
  stop();
}

void UdsServer::start() {
  if (running_.load()) return;
  running_.store(true);
  thread_ = std::thread(&UdsServer::run, this);
}

void UdsServer::stop() {
  if (!running_.load()) return;
  running_.store(false);
  if (server_fd_ >= 0) {
    ::shutdown(server_fd_, SHUT_RDWR);
    ::close(server_fd_);
    server_fd_ = -1;
  }
  if (thread_.joinable()) {
    thread_.join();
  }
  std::filesystem::remove(socket_path_);
}

void UdsServer::run() {
  std::filesystem::path p(socket_path_);
  if (p.has_parent_path()) {
    std::filesystem::create_directories(p.parent_path());
  }
  std::filesystem::remove(socket_path_);

  server_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    std::cerr << "chat2me-motion: failed to create unix socket: "
              << std::strerror(errno) << '\n';
    return;
  }

  struct sockaddr_un addr {};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

  if (::bind(server_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    std::cerr << "chat2me-motion: failed to bind socket " << socket_path_
              << ": " << std::strerror(errno) << '\n';
    ::close(server_fd_);
    server_fd_ = -1;
    return;
  }

  ::chmod(socket_path_.c_str(), 0666);

  if (::listen(server_fd_, 16) < 0) {
    std::cerr << "chat2me-motion: listen failed: " << std::strerror(errno) << '\n';
    ::close(server_fd_);
    server_fd_ = -1;
    return;
  }

  std::cout << "chat2me-motion: UDS HTTP server listening at " << socket_path_ << '\n';

  while (running_.load()) {
    const int client_fd = ::accept(server_fd_, nullptr, nullptr);
    if (client_fd < 0) {
      if (!running_.load()) break;
      if (errno == EINTR || errno == EAGAIN) continue;
      break;
    }
    std::thread(&UdsServer::handleClient, this, client_fd).detach();
  }
}

void UdsServer::handleClient(int client_fd) {
  std::string request;
  char buf[4096];
  while (true) {
    const auto n = ::read(client_fd, buf, sizeof(buf));
    if (n <= 0) break;
    request.append(buf, n);
    if (request.find("\r\n\r\n") != std::string::npos ||
        request.find("\n\n") != std::string::npos) {
      // Check if body complete if Content-Length present
      const auto cl_pos = request.find("Content-Length:");
      if (cl_pos != std::string::npos) {
        auto val_pos = request.find_first_not_of(" \t", cl_pos + 15);
        auto end_pos = request.find_first_of("\r\n", val_pos);
        if (val_pos != std::string::npos && end_pos != std::string::npos) {
          int cl = std::stoi(request.substr(val_pos, end_pos - val_pos));
          auto body_sep = request.find("\r\n\r\n");
          size_t body_start = (body_sep != std::string::npos) ? (body_sep + 4) : request.find("\n\n") + 2;
          if (request.size() >= body_start + cl) {
            break;
          }
          continue;
        }
      }
      break;
    }
  }

  // Parse HTTP method and body
  std::string response_body;
  int status_code = 202;
  std::string status_text = "Accepted";

  auto body_sep = request.find("\r\n\r\n");
  size_t body_start = (body_sep != std::string::npos) ? (body_sep + 4) : (request.find("\n\n") != std::string::npos ? request.find("\n\n") + 2 : std::string::npos);
  std::string body = (body_start != std::string::npos) ? request.substr(body_start) : "";

  if (request.rfind("GET /health", 0) == 0 || request.find("GET /health ") != std::string::npos) {
    status_code = 200;
    status_text = "OK";
    response_body = "{\"status\":\"ok\"}";
  } else {
    // Treat as action execution request
    ActionGoal goal;
    parseJsonValue(body, "id", goal.id);
    parseJsonValue(body, "name", goal.name);

    std::string timeout_str;
    if (parseJsonValue(body, "timeout_ms", timeout_str)) {
      try { goal.timeout_ms = std::stoi(timeout_str); } catch (...) {}
    }
    std::string preempt_str;
    if (parseJsonValue(body, "preempt", preempt_str)) {
      goal.preempt = (preempt_str == "true" || preempt_str == "1");
    }
    std::string dir_str;
    if (parseJsonValue(body, "direction_deg", dir_str)) {
      try {
        goal.direction_deg = std::stod(dir_str);
        goal.has_direction = true;
      } catch (...) {}
    }

    if (!goal.name.empty()) {
      controller_->enqueueAction(goal);
      response_body = "{\"ok\":true,\"id\":\"" + goal.id + "\",\"action\":\"" + goal.name + "\"}";
    } else {
      status_code = 400;
      status_text = "Bad Request";
      response_body = "{\"error\":\"missing action name\"}";
    }
  }

  std::ostringstream ss;
  ss << "HTTP/1.1 " << status_code << " " << status_text << "\r\n"
     << "Content-Type: application/json; charset=utf-8\r\n"
     << "Content-Length: " << response_body.size() << "\r\n"
     << "Connection: close\r\n\r\n"
     << response_body;

  const std::string resp_str = ss.str();
  size_t written = 0;
  while (written < resp_str.size()) {
    const auto n = ::write(client_fd, resp_str.data() + written, resp_str.size() - written);
    if (n <= 0) break;
    written += n;
  }
  ::close(client_fd);
}

}  // namespace chat2me_motion
