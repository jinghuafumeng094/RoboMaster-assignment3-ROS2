#include "hikrobot_camera/camera_node.hpp"

#include <vector>

namespace hikrobot_camera
{

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{
  RCLCPP_INFO(get_logger(), "hikrobot_camera node starting...");

  std::vector<MV_CC_DEVICE_INFO> devices;
  if (!mvs_camera_.listDevices(devices)) {
    RCLCPP_ERROR(get_logger(), "Failed to enumerate MVS devices");
    return;
  }

  RCLCPP_INFO(get_logger(), "Found %zu device(s)", devices.size());

  for (size_t i = 0; i < devices.size(); ++i) {
    const auto & info = devices[i];
    if (info.nTLayerType == MV_USB_DEVICE) {
      RCLCPP_INFO(
        get_logger(), "[%zu] USB | Model: %s | Serial: %s",
        i,
        info.SpecialInfo.stUsb3VInfo.chModelName,
        info.SpecialInfo.stUsb3VInfo.chSerialNumber);
    } else if (info.nTLayerType == MV_GIGE_DEVICE) {
      uint32_t ip = info.SpecialInfo.stGigEInfo.nCurrentIp;
      RCLCPP_INFO(
        get_logger(), "[%zu] GigE | Model: %s | IP: %u.%u.%u.%u",
        i,
        info.SpecialInfo.stGigEInfo.chModelName,
        (ip >> 24) & 0xFF, (ip >> 16) & 0xFF, (ip >> 8) & 0xFF, ip & 0xFF);
    }
  }
}

}  // namespace hikrobot_camera