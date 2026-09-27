#include <memory>

#include "hikrobot_camera/camera_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "MvCameraControl.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<hikrobot_camera::CameraNode>();

  if (!node->isInitialized()) {
    RCLCPP_FATAL(rclcpp::get_logger("main"),
      "Camera node failed to initialize, exiting.");
    // 主动析构，触发 ~CameraNode → close()，释放相机资源
    node.reset();
    rclcpp::shutdown();
    MV_CC_Finalize();
    return 1;
  }

  rclcpp::spin(node);

  // 关键顺序：
  // 1. 停止 ROS 执行
  rclcpp::shutdown();
  // 2. 主动析构 node，触发 ~CameraNode → MvsCamera::close() → 释放相机句柄
  node.reset();
  // 3. 相机句柄已经释放，此时再反初始化 SDK
  MV_CC_Finalize();

  return 0;
}