cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 查找 MVS SDK
find_path(MVS_INCLUDE_DIR MvCameraControl.h PATHS /opt/MVS/include)
find_library(MVS_LIBRARY NAMES MvCameraControl PATHS /opt/MVS/lib/64)
if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
  message(FATAL_ERROR "MVS SDK not found! Please check /opt/MVS/")
endif()

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

# TODO(student): Declare and link the dependencies used by your implementation,
# including the vendor SDK. The starter does not discover or link MVS.

install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
