cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# MVS SDK 路径：优先用环境变量 MVS_ROOT，默认 /opt/MVS
if(NOT DEFINED MVS_ROOT)
  if(DEFINED ENV{MVS_ROOT})
    set(MVS_ROOT $ENV{MVS_ROOT})
  else()
    set(MVS_ROOT "/opt/MVS")
  endif()
endif()

find_path(MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  PATHS ${MVS_ROOT}/include
  NO_DEFAULT_PATH
)

find_library(MVS_LIBRARY
  NAMES MvCameraControl
  PATHS ${MVS_ROOT}/lib/64
  NO_DEFAULT_PATH
)

if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
  message(FATAL_ERROR
    "MVS SDK not found under ${MVS_ROOT}. "
    "Install HIKROBOT MVS SDK or set MVS_ROOT.")
endif()

message(STATUS "MVS include: ${MVS_INCLUDE_DIR}")
message(STATUS "MVS library: ${MVS_LIBRARY}")

add_executable(camera_node
  src/main.cpp
  src/camera_node.cpp
  src/mvs_camera.cpp
)
target_compile_features(camera_node PRIVATE cxx_std_17)
target_include_directories(camera_node PRIVATE
  include
  ${MVS_INCLUDE_DIR}
)
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(camera_node ${MVS_LIBRARY})

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(camera_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()