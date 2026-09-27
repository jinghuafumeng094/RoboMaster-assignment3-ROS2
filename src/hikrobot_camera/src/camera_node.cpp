#include "hikrobot_camera/camera_node.hpp"

#include <vector>

namespace hikrobot_camera
{

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{
  RCLCPP_INFO(get_logger(), "hikrobot_camera node starting...");

  if (!mvs_camera_.openByIndex(0)) {
    RCLCPP_ERROR(get_logger(), "Failed to open camera index 0");
    return;
  }
  RCLCPP_INFO(get_logger(), "Camera opened");

  if (!mvs_camera_.startGrabbing()) {
    RCLCPP_ERROR(get_logger(), "Failed to start grabbing");
    return;
  }
  RCLCPP_INFO(get_logger(), "Grabbing started");

  for (int i = 0; i < 10; ++i) {
    FrameInfo frame;
    if (!mvs_camera_.getFrame(frame, 1000)) {
      RCLCPP_WARN(get_logger(), "Frame %d: timeout", i);
      continue;
    }

    RCLCPP_INFO(
      get_logger(),
      "Frame %u: %ux%u, len=%u, pixel_type=0x%x",
      frame.frame_num, frame.width, frame.height,
      frame.data_len, static_cast<unsigned int>(frame.pixel_type));

    mvs_camera_.releaseFrame(frame);
  }

  mvs_camera_.stopGrabbing();
  mvs_camera_.close();
  RCLCPP_INFO(get_logger(), "Camera closed. Test done.");
}

}  // namespace hikrobot_camera