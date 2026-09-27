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
    rclcpp::shutdown();
    MV_CC_Finalize();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  MV_CC_Finalize();
  return 0;
}