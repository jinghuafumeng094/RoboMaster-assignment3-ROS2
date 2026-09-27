cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ---------- MVS SDK 查找 ----------
# MVS_ROOT：SDK 根目录，默认 /opt/MVS，可用环境变量覆盖
if(NOT DEFINED MVS_ROOT)
  if(DEFINED ENV{MVS_ROOT})
    set(MVS_ROOT $ENV{MVS_ROOT})
  else()
    set(MVS_ROOT "/opt/MVS")
  endif()
endif()

# MVS_LIB_SUBDIR：库子目录名，x86_64 上是 "64"，ARM 上可能是 "aarch64"
if(NOT DEFINED MVS_LIB_SUBDIR)
  if(DEFINED ENV{MVS_LIB_SUBDIR})
    set(MVS_LIB_SUBDIR $ENV{MVS_LIB_SUBDIR})
  else()
    set(MVS_LIB_SUBDIR "64")
  endif()
endif()

find_path(MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  PATHS ${MVS_ROOT}/include
  NO_DEFAULT_PATH
)

find_library(MVS_LIBRARY
  NAMES MvCameraControl
  PATHS ${MVS_ROOT}/lib/${MVS_LIB_SUBDIR}
  NO_DEFAULT_PATH
)

if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
  message(FATAL_ERROR
    "MVS SDK not found under ${MVS_ROOT} (lib subdir: ${MVS_LIB_SUBDIR}). "
    "Install HIKROBOT MVS SDK, or set MVS_ROOT / MVS_LIB_SUBDIR.")
endif()

message(STATUS "MVS include: ${MVS_INCLUDE_DIR}")
message(STATUS "MVS library: ${MVS_LIBRARY}")

# ---------- 可执行文件 ----------
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
)
target_link_libraries(camera_node ${MVS_LIBRARY})

# ---------- rpath：让可执行文件运行时能找到 libMvCameraControl.so ----------
get_filename_component(MVS_LIBRARY_DIR "${MVS_LIBRARY}" DIRECTORY)
set_target_properties(camera_node PROPERTIES
  BUILD_RPATH "${MVS_LIBRARY_DIR}"
  INSTALL_RPATH "${MVS_LIBRARY_DIR}"
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(camera_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------- 安装 ----------
install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()