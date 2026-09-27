#include "hikrobot_camera/camera_node.hpp"

#include <chrono>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

namespace hikrobot_camera
{

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{
  RCLCPP_INFO(get_logger(), "hikrobot_camera node starting...");

  // ---------- 声明参数 ----------
  rcl_interfaces::msg::ParameterDescriptor desc_exp;
  desc_exp.description = "Exposure time in microseconds";
  this->declare_parameter<double>("exposure_time", 5000.0, desc_exp);

  rcl_interfaces::msg::ParameterDescriptor desc_gain;
  desc_gain.description = "Gain in dB";
  this->declare_parameter<double>("gain", 0.0, desc_gain);

  rcl_interfaces::msg::ParameterDescriptor desc_fr;
  desc_fr.description = "Target acquisition frame rate in Hz";
  this->declare_parameter<double>("frame_rate", 30.0, desc_fr);

  rcl_interfaces::msg::ParameterDescriptor desc_topic;
  desc_topic.description = "Image topic name to publish";
  this->declare_parameter<std::string>("topic_name", "image_raw", desc_topic);

  // ---------- 读取参数 ----------
  topic_name_ = this->get_parameter("topic_name").as_string();

  // ---------- 打开相机 ----------
  if (!mvs_camera_.openByIndex(0)) {
    RCLCPP_ERROR(get_logger(), "Failed to open camera index 0");
    return;
  }
  RCLCPP_INFO(get_logger(), "Camera opened");

  // ---------- 应用初始参数 ----------
  double exp = this->get_parameter("exposure_time").as_double();
  double g   = this->get_parameter("gain").as_double();
  double fr  = this->get_parameter("frame_rate").as_double();

  if (!mvs_camera_.setExposureTime(exp)) {
    RCLCPP_WARN(get_logger(), "Failed to set initial exposure time %.1f us", exp);
  } else {
    RCLCPP_INFO(get_logger(), "Exposure time initialized to %.1f us", exp);
  }

  if (!mvs_camera_.setGain(g)) {
    RCLCPP_WARN(get_logger(), "Failed to set initial gain %.2f dB", g);
  } else {
    RCLCPP_INFO(get_logger(), "Gain initialized to %.2f dB", g);
  }

  if (!mvs_camera_.setFrameRate(fr)) {
    RCLCPP_WARN(get_logger(), "Failed to set initial frame rate %.2f Hz", fr);
  } else {
    RCLCPP_INFO(get_logger(), "Frame rate initialized to %.2f Hz", fr);
  }

  // ---------- 取流 ----------
  if (!mvs_camera_.startGrabbing()) {
    RCLCPP_ERROR(get_logger(), "Failed to start grabbing");
    return;
  }
  RCLCPP_INFO(get_logger(), "Grabbing started");

  // ---------- 发布者（用参数里的 topic_name） ----------
  publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);
  RCLCPP_INFO(get_logger(), "Publishing to /%s", topic_name_.c_str());

  // ---------- 定时器 ----------
  timer_ = this->create_wall_timer(
    10ms, std::bind(&CameraNode::timerCallback, this));

  // ---------- 参数回调 ----------
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CameraNode::onParameterChange, this, std::placeholders::_1));
}

CameraNode::~CameraNode()
{
  if (timer_) {
    timer_->cancel();
  }
  mvs_camera_.stopGrabbing();
  mvs_camera_.close();
  RCLCPP_INFO(get_logger(), "Camera closed.");
}

rcl_interfaces::msg::SetParametersResult CameraNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : params) {
    const std::string & name = p.get_name();

    if (name == "exposure_time") {
      double v = p.as_double();
      if (v <= 0.0) {
        result.successful = false;
        result.reason = "exposure_time must be > 0";
        return result;
      }
      if (!mvs_camera_.setExposureTime(v)) {
        result.successful = false;
        result.reason = "Failed to set exposure time on camera";
        return result;
      }
      RCLCPP_INFO(get_logger(), "Exposure time -> %.1f us", v);

    } else if (name == "gain") {
      double v = p.as_double();
      if (v < 0.0) {
        result.successful = false;
        result.reason = "gain must be >= 0";
        return result;
      }
      if (!mvs_camera_.setGain(v)) {
        result.successful = false;
        result.reason = "Failed to set gain on camera";
        return result;
      }
      RCLCPP_INFO(get_logger(), "Gain -> %.2f dB", v);

    } else if (name == "frame_rate") {
      double v = p.as_double();
      if (v <= 0.0) {
        result.successful = false;
        result.reason = "frame_rate must be > 0";
        return result;
      }
      if (!mvs_camera_.setFrameRate(v)) {
        result.successful = false;
        result.reason = "Failed to set frame rate on camera";
        return result;
      }
      RCLCPP_INFO(get_logger(), "Frame rate -> %.2f Hz", v);

    } else if (name == "topic_name") {
      const std::string new_name = p.as_string();
      if (new_name.empty()) {
        result.successful = false;
        result.reason = "topic_name must not be empty";
        return result;
      }
      topic_name_ = new_name;
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);
      RCLCPP_INFO(get_logger(), "Topic name -> /%s (new publisher created)", new_name.c_str());
    }
  }
  return result;
}

void CameraNode::timerCallback()
{
  FrameInfo frame;
  if (!mvs_camera_.getFrame(frame, 100)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Frame timeout");
    return;
  }

  std::vector<uint8_t> bgr;
  uint32_t w = 0;
  uint32_t h = 0;
  if (!mvs_camera_.convertToBgr(frame, bgr, w, h)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Pixel convert failed");
    mvs_camera_.releaseFrame(frame);
    return;
  }

  auto img = std::make_unique<sensor_msgs::msg::Image>();
  img->header.stamp = this->now();
  img->header.frame_id = frame_id_;
  img->height = h;
  img->width  = w;
  img->encoding = "bgr8";
  img->is_bigendian = 0;
  img->step = w * 3;
  img->data = std::move(bgr);

  publisher_->publish(std::move(img));

  mvs_camera_.releaseFrame(frame);
}

}  // namespace hikrobot_camera