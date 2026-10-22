cmake_minimum_required(VERSION 3.16)
project(chat2me_motion LANGUAGES CXX)

set(CHAT2ME_MOTION_VERSION "" CACHE STRING "Runtime version shown by chat2me-motion")
if(NOT CHAT2ME_MOTION_VERSION)
  set(CHAT2ME_MOTION_COMMIT_DATE "")
  execute_process(
    COMMAND git log -1 --format=%cI
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE CHAT2ME_MOTION_COMMIT_DATE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(CHAT2ME_MOTION_COMMIT_DATE)
    execute_process(
      COMMAND date -u -d "${CHAT2ME_MOTION_COMMIT_DATE}" +%Y%m%d
      OUTPUT_VARIABLE CHAT2ME_MOTION_VERSION
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
  endif()
  if(NOT CHAT2ME_MOTION_VERSION)
    string(TIMESTAMP CHAT2ME_MOTION_VERSION "%Y%m%d" UTC)
  endif()
  set(CHAT2ME_MOTION_VERSION "${CHAT2ME_MOTION_VERSION}" CACHE STRING
      "Runtime version shown by chat2me-motion" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Embedded RPATH allows running binaries on ROS 2 Humble targets without sourcing setup.bash
set(CMAKE_BUILD_RPATH "/opt/ros/humble/lib;/opt/ros/humble/lib/aarch64-linux-gnu;/opt/ros/humble/lib/x86_64-linux-gnu")
set(CMAKE_INSTALL_RPATH "/opt/ros/humble/lib;/opt/ros/humble/lib/aarch64-linux-gnu;/opt/ros/humble/lib/x86_64-linux-gnu")
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(chat2me_motion_core STATIC
  src/config.cpp
  src/motion_controller.cpp
  src/uds_server.cpp
  src/service.cpp
)

target_include_directories(chat2me_motion_core PUBLIC include)
target_compile_options(chat2me_motion_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
ament_target_dependencies(chat2me_motion_core rclcpp sensor_msgs std_msgs)

add_executable(chat2me-motion
  src/main.cpp
)

set_target_properties(chat2me-motion PROPERTIES
  BUILD_RPATH "/opt/ros/humble/lib;/opt/ros/humble/lib/aarch64-linux-gnu;/opt/ros/humble/lib/x86_64-linux-gnu"
  INSTALL_RPATH "/opt/ros/humble/lib;/opt/ros/humble/lib/aarch64-linux-gnu;/opt/ros/humble/lib/x86_64-linux-gnu")

target_include_directories(chat2me-motion PRIVATE include)
target_link_libraries(chat2me-motion chat2me_motion_core)
ament_target_dependencies(chat2me-motion rclcpp sensor_msgs std_msgs)
target_link_options(chat2me-motion PRIVATE -Wl,--disable-new-dtags)
target_compile_definitions(chat2me-motion PRIVATE
  CHAT2ME_MOTION_VERSION="${CHAT2ME_MOTION_VERSION}")
target_compile_options(chat2me-motion PRIVATE -Wall -Wextra -Wpedantic -Werror)

install(TARGETS chat2me-motion RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES config/config.example.yml DESTINATION share/${PROJECT_NAME})

ament_package()
