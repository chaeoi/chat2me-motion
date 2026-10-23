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

option(CHAT2ME_MOTION_WITH_ROS "Build the ROS 2 runtime" ON)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
include(CTest)

add_library(chat2me_motion_core STATIC
  src/config.cpp
  src/motion_engine.cpp
  src/protocol.cpp
  src/uds_server.cpp
  src/service.cpp
)

target_include_directories(chat2me_motion_core PUBLIC include)
target_include_directories(chat2me_motion_core SYSTEM PUBLIC third_party)
target_link_libraries(chat2me_motion_core PUBLIC yaml-cpp Threads::Threads)
target_compile_options(chat2me_motion_core PRIVATE -Wall -Wextra -Wpedantic -Werror)

if(BUILD_TESTING)
  add_executable(motion_offline_test tests/offline_test.cpp)
  target_link_libraries(motion_offline_test PRIVATE chat2me_motion_core)
  target_compile_options(motion_offline_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
  add_test(NAME motion_offline COMMAND motion_offline_test)
  add_executable(motion_offline_server tests/offline_server.cpp)
  target_link_libraries(motion_offline_server PRIVATE chat2me_motion_core)
  target_compile_options(motion_offline_server PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

if(CHAT2ME_MOTION_WITH_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)

  add_executable(chat2me-motion
    src/main.cpp
    src/motion_controller.cpp
  )

  set_target_properties(chat2me-motion PROPERTIES
    BUILD_RPATH "/opt/ros/humble/lib;/opt/ros/humble/lib/aarch64-linux-gnu;/opt/ros/humble/lib/x86_64-linux-gnu"
    INSTALL_RPATH "/opt/ros/humble/lib;/opt/ros/humble/lib/aarch64-linux-gnu;/opt/ros/humble/lib/x86_64-linux-gnu")

  target_include_directories(chat2me-motion PRIVATE include)
  target_link_libraries(chat2me-motion chat2me_motion_core)
  ament_target_dependencies(chat2me-motion rclcpp sensor_msgs)
  target_link_options(chat2me-motion PRIVATE -Wl,--disable-new-dtags)
  target_compile_definitions(chat2me-motion PRIVATE
    CHAT2ME_MOTION_VERSION="${CHAT2ME_MOTION_VERSION}")
  target_compile_options(chat2me-motion PRIVATE -Wall -Wextra -Wpedantic -Werror)

  install(TARGETS chat2me-motion RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(FILES config/config.example.yml DESTINATION share/${PROJECT_NAME})
  install(FILES third_party/NOTICE DESTINATION share/${PROJECT_NAME}/licenses)

  ament_package()
endif()
