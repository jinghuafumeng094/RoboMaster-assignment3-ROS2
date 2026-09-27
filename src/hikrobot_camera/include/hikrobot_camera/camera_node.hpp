#ifndef HIKROBOT_CAMERA__CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA__CAMERA_NODE_HPP_

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "hikrobot_camera/mvs_camera.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace hikrobot_camera
{

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode() override;

  bool isInitialized() const { return initialized_; }

private:
  void timerCallback();
  void reconnectLoop();
  bool tryReconnect();

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params);

  MvsCamera mvs_camera_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  std::string frame_id_     = "camera_optical_frame";
  std::string topic_name_   = "image_raw";
  std::string pixel_format_ = "bgr8";
  std::string serial_number_ = "";
  std::string ip_address_    = "";

  // ---------- 断线重连 ----------
  std::thread reconnect_thread_;
  std::atomic<bool> stop_reconnect_{false};
  std::atomic<bool> reconnect_needed_{false};
  std::atomic<bool> reconnecting_{false};
  std::atomic<int> consecutive_failures_{0};

  bool initialized_ = false;
};

}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__CAMERA_NODE_HPP_