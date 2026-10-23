#include "chat2me_motion/uds_server.hpp"
#include "chat2me_motion/protocol.hpp"

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <stdexcept>

namespace chat2me_motion {
namespace {
constexpr std::size_t kHeaderLimit = 8192;
constexpr std::size_t kBodyLimit = 16384;

double nowSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Client {
  int fd;
  ~Client() { ::close(fd); }
};

void sendResponse(int fd, const HttpResponse &response) {
  const auto text = "HTTP/1.1 " + std::to_string(response.status) + " Response\r\n"
      "Content-Type: application/json\r\nConnection: close\r\nContent-Length: " +
      std::to_string(response.body.size()) + "\r\n\r\n" + response.body;
  std::size_t offset = 0;
  while (offset < text.size()) {
    auto count = ::send(fd, text.data() + offset, text.size() - offset, MSG_NOSIGNAL);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) break;
    offset += static_cast<std::size_t>(count);
  }
}

std::string lower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return std::tolower(c); });
  return text;
}
}  // namespace

UdsServer::UdsServer(std::string socket_path, MotionEngine &engine)
    : socket_path_(std::move(socket_path)), engine_(engine) {}

UdsServer::~UdsServer() { stop(); }

void UdsServer::start() {
  if (running_.load()) return;
  if (socket_path_.empty() || socket_path_.size() >= sizeof(sockaddr_un::sun_path))
    throw std::runtime_error("invalid Unix socket path");
  const auto parent = std::filesystem::path(socket_path_).parent_path();
  if (!parent.empty()) std::filesystem::create_directories(parent);
  struct sockaddr_un address {};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, socket_path_.c_str(), socket_path_.size() + 1);
  struct stat existing {};
  if (::lstat(socket_path_.c_str(), &existing) == 0) {
    if (!S_ISSOCK(existing.st_mode)) throw std::runtime_error("socket path already contains a non-socket file");
    Client probe{::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0)};
    if (probe.fd < 0) throw std::runtime_error("cannot probe existing Unix socket");
    if (::connect(probe.fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0)
      throw std::runtime_error("motion socket is already in use");
    if (errno != ECONNREFUSED && errno != ENOENT) throw std::runtime_error("cannot inspect existing Unix socket");
    if (::unlink(socket_path_.c_str()) != 0 && errno != ENOENT) throw std::runtime_error("cannot remove stale Unix socket");
  }
  server_fd_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (server_fd_ < 0) throw std::runtime_error("cannot create Unix socket");
  if (::bind(server_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
    ::close(server_fd_);
    server_fd_ = -1;
    throw std::runtime_error("cannot bind motion socket");
  }
  if (::chmod(socket_path_.c_str(), 0660) != 0 || ::listen(server_fd_, 16) != 0) {
    ::close(server_fd_);
    server_fd_ = -1;
    ::unlink(socket_path_.c_str());
    throw std::runtime_error("cannot listen on motion socket");
  }
  running_.store(true);
  try {
    thread_ = std::thread(&UdsServer::run, this);
  } catch (...) {
    stop();
    throw;
  }
}

void UdsServer::stop() {
  running_.store(false);
  if (server_fd_ >= 0) ::shutdown(server_fd_, SHUT_RDWR);
  if (thread_.joinable()) thread_.join();
  if (server_fd_ >= 0) {
    ::close(server_fd_);
    server_fd_ = -1;
    ::unlink(socket_path_.c_str());
  }
}

void UdsServer::run() {
  while (running_.load()) {
    const int fd = ::accept4(server_fd_, nullptr, nullptr, SOCK_CLOEXEC);
    if (fd < 0) {
      if (errno == EINTR) continue;
      break;
    }
    Client client{fd};
    timeval timeout{1, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    try {
      handleClient(fd);
    } catch (const std::exception &) {
      sendResponse(fd, {400, "{\"ok\":false,\"error\":\"invalid HTTP request\"}"});
    }
  }
}

void UdsServer::handleClient(int client_fd) {
  std::string request;
  char buffer[4096];
  const double deadline = nowSeconds() + 2.0;
  auto receive = [&]() {
    if (!running_.load() || nowSeconds() > deadline) throw std::runtime_error("request timeout");
    auto count = ::recv(client_fd, buffer, sizeof(buffer), 0);
    if (count <= 0) throw std::runtime_error("incomplete request");
    request.append(buffer, static_cast<std::size_t>(count));
    if (request.size() > kHeaderLimit + kBodyLimit) throw std::runtime_error("request too large");
  };
  std::size_t separator;
  while ((separator = request.find("\r\n\r\n")) == std::string::npos) {
    if (request.size() > kHeaderLimit) throw std::runtime_error("headers too large");
    receive();
  }
  if (separator > kHeaderLimit) throw std::runtime_error("headers too large");
  std::istringstream headers(request.substr(0, separator));
  std::string line, method, path, version, extra;
  std::getline(headers, line);
  std::istringstream first(line);
  if (!(first >> method >> path >> version) || (first >> extra) ||
      (version != "HTTP/1.1" && version != "HTTP/1.0")) throw std::runtime_error("invalid request line");
  std::size_t length = 0;
  bool has_length = false;
  while (std::getline(headers, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    auto colon = line.find(':');
    if (colon == std::string::npos) throw std::runtime_error("invalid HTTP header");
    auto name = lower(line.substr(0, colon));
    if (name == "transfer-encoding") throw std::runtime_error("chunked requests are not supported");
    if (name != "content-length") continue;
    if (has_length) throw std::runtime_error("duplicate content length");
    has_length = true;
    auto start = line.find_first_not_of(" \t", colon + 1);
    if (start == std::string::npos) throw std::runtime_error("missing content length");
    auto end = line.find_last_not_of(" \t");
    const char *begin = line.data() + start;
    const char *last = line.data() + end + 1;
    const auto parsed = std::from_chars(begin, last, length);
    if (parsed.ec != std::errc{} || parsed.ptr != last || length > kBodyLimit)
      throw std::runtime_error("invalid content length");
  }
  if (method == "POST" && !has_length) throw std::runtime_error("content length required");
  const auto body_start = separator + 4;
  while (request.size() < body_start + length) receive();
  sendResponse(client_fd, handleRequest(method, path, request.substr(body_start, length), engine_, nowSeconds()));
}

}  // namespace chat2me_motion
