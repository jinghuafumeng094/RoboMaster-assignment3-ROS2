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

  // 声明参数
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

  rcl_interfaces::msg::ParameterDescriptor desc_fmt;
  desc_fmt.description = "Pixel format: bgr8 or bayer_rggb8";
  this->declare_parameter<std::string>("pixel_format", "bgr8", desc_fmt);

  rcl_interfaces::msg::ParameterDescriptor desc_serial;
  desc_serial.description = "Camera serial number. If empty, fall back to index.";
  this->declare_parameter<std::string>("serial_number", "", desc_serial);

  rcl_interfaces::msg::ParameterDescriptor desc_ip;
  desc_ip.description = "Camera IP address (GigE only). If empty, ignored.";
  this->declare_parameter<std::string>("ip_address", "", desc_ip);

  rcl_interfaces::msg::ParameterDescriptor desc_frame;
  desc_frame.description = "frame_id for published images";
  this->declare_parameter<std::string>("frame_id", "camera_optical_frame", desc_frame);

  // 读取参数
  topic_name_    = this->get_parameter("topic_name").as_string();
  pixel_format_  = this->get_parameter("pixel_format").as_string();
  serial_number_ = this->get_parameter("serial_number").as_string();
  ip_address_    = this->get_parameter("ip_address").as_string();
  frame_id_      = this->get_parameter("frame_id").as_string();

  // 打开相机 
  bool opened = false;
  if (!ip_address_.empty()) {
    RCLCPP_INFO(get_logger(), "Trying to open camera by IP: %s", ip_address_.c_str());
    opened = mvs_camera_.openByIp(ip_address_);
    if (!opened) {
      RCLCPP_FATAL(get_logger(), "Failed to open camera by IP '%s': %s",
        ip_address_.c_str(), mvs_camera_.lastError().c_str());
      return;
    }
  } else if (!serial_number_.empty()) {
    RCLCPP_INFO(get_logger(), "Trying to open camera by serial: %s", serial_number_.c_str());
    opened = mvs_camera_.openBySerial(serial_number_);
    if (!opened) {
      RCLCPP_FATAL(get_logger(), "Failed to open camera by serial '%s': %s",
        serial_number_.c_str(), mvs_camera_.lastError().c_str());
      return;
    }
  } else {
    RCLCPP_INFO(get_logger(), "No serial/IP specified, opening camera index 0");
    opened = mvs_camera_.openByIndex(0);
    if (!opened) {
      RCLCPP_FATAL(get_logger(), "Failed to open camera index 0: %s",
        mvs_camera_.lastError().c_str());
      return;
    }
  }
  RCLCPP_INFO(get_logger(), "Camera opened");

  // 应用初始参数
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

  // 取流 
  if (!mvs_camera_.startGrabbing()) {
    RCLCPP_FATAL(get_logger(), "Failed to start grabbing");
    return;
  }
  RCLCPP_INFO(get_logger(), "Grabbing started");

  // 发布者 
  publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);
  RCLCPP_INFO(get_logger(), "Publishing to /%s, pixel_format=%s, frame_id=%s",
    topic_name_.c_str(), pixel_format_.c_str(), frame_id_.c_str());

  // 定时器
  timer_ = this->create_wall_timer(
    10ms, std::bind(&CameraNode::timerCallback, this));

  // 参数回调
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CameraNode::onParameterChange, this, std::placeholders::_1));

  // 启动重连线程
  reconnect_thread_ = std::thread(&CameraNode::reconnectLoop, this);

  initialized_ = true;
  RCLCPP_INFO(get_logger(), "hikrobot_camera node initialized successfully.");
}

CameraNode::~CameraNode()
{
  stop_reconnect_ = true;
  if (reconnect_thread_.joinable()) {
    reconnect_thread_.join();
  }

  if (timer_) {
    timer_->cancel();
  }
  mvs_camera_.stopGrabbing();
  mvs_camera_.close();
  RCLCPP_INFO(get_logger(), "Camera closed.");
}

void CameraNode::reconnectLoop()
{
  while (!stop_reconnect_.load()) {
    if (!reconnect_needed_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }

    reconnecting_ = true;
    RCLCPP_WARN(get_logger(), "Attempting to reconnect camera...");

    bool ok = tryReconnect();

    if (ok) {
      RCLCPP_INFO(get_logger(), "Camera reconnected successfully");
      reconnect_needed_ = false;
      consecutive_failures_ = 0;
    } else {
      RCLCPP_WARN(get_logger(), "Reconnect failed, will retry in 1s");
    }

    reconnecting_ = false;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  RCLCPP_INFO(get_logger(), "Reconnect thread exiting");
}

bool CameraNode::tryReconnect()
{
  mvs_camera_.close();

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  bool ok = false;
  if (!ip_address_.empty()) {
    ok = mvs_camera_.openByIp(ip_address_);
  } else if (!serial_number_.empty()) {
    ok = mvs_camera_.openBySerial(serial_number_);
  } else {
    ok = mvs_camera_.openByIndex(0);
  }
  if (!ok) {
    RCLCPP_WARN(get_logger(), "Reconnect: open failed: %s",
      mvs_camera_.lastError().c_str());
    return false;
  }

  double exp = this->get_parameter("exposure_time").as_double();
  double g   = this->get_parameter("gain").as_double();
  double fr  = this->get_parameter("frame_rate").as_double();

  if (!mvs_camera_.setExposureTime(exp)) {
    RCLCPP_WARN(get_logger(), "Reconnect: set exposure %.1f failed", exp);
  }
  if (!mvs_camera_.setGain(g)) {
    RCLCPP_WARN(get_logger(), "Reconnect: set gain %.2f failed", g);
  }
  if (!mvs_camera_.setFrameRate(fr)) {
    RCLCPP_WARN(get_logger(), "Reconnect: set frame_rate %.2f failed", fr);
  }

  if (!mvs_camera_.startGrabbing()) {
    RCLCPP_WARN(get_logger(), "Reconnect: startGrabbing failed");
    mvs_camera_.close();
    return false;
  }

  return true;
}

rcl_interfaces::msg::SetParametersResult CameraNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // 第一遍：字符串参数（不需要回滚）
  for (const auto & p : params) {
    const std::string & name = p.get_name();

    if (name == "topic_name") {
      const std::string new_name = p.as_string();
      if (new_name.empty()) {
        result.successful = false;
        result.reason = "topic_name must not be empty";
        return result;
      }
      topic_name_ = new_name;
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);
      RCLCPP_INFO(get_logger(), "Topic name -> /%s", new_name.c_str());

    } else if (name == "pixel_format") {
      const std::string fmt = p.as_string();
      if (fmt != "bgr8" && fmt != "bayer_rggb8") {
        result.successful = false;
        result.reason = "pixel_format must be 'bgr8' or 'bayer_rggb8'";
        return result;
      }
      pixel_format_ = fmt;
      RCLCPP_INFO(get_logger(), "Pixel format -> %s", fmt.c_str());

    } else if (name == "frame_id") {
      const std::string fid = p.as_string();
      if (fid.empty()) {
        result.successful = false;
        result.reason = "frame_id must not be empty";
        return result;
      }
      frame_id_ = fid;
      RCLCPP_INFO(get_logger(), "frame_id -> %s", fid.c_str());
    }
  }

  // 第二遍：数值参数，带回滚
  double old_exp = 0.0;
  double old_gain = 0.0;
  double old_fr = 0.0;
  mvs_camera_.getExposureTime(old_exp);
  mvs_camera_.getGain(old_gain);
  mvs_camera_.getFrameRate(old_fr);

  bool has_exp = false;
  bool has_gain = false;
  bool has_fr = false;
  double new_exp = old_exp;
  double new_gain = old_gain;
  double new_fr = old_fr;

  for (const auto & p : params) {
    const std::string & name = p.get_name();
    if (name == "exposure_time") {
      double v = p.as_double();
      if (v <= 0.0) {
        result.successful = false;
        result.reason = "exposure_time must be > 0";
        return result;
      }
      new_exp = v;
      has_exp = true;
    } else if (name == "gain") {
      double v = p.as_double();
      if (v < 0.0) {
        result.successful = false;
        result.reason = "gain must be >= 0";
        return result;
      }
      new_gain = v;
      has_gain = true;
    } else if (name == "frame_rate") {
      double v = p.as_double();
      if (v <= 0.0) {
        result.successful = false;
        result.reason = "frame_rate must be > 0";
        return result;
      }
      new_fr = v;
      has_fr = true;
    }
  }

    // ★ rollback 现在返回失败信息（空字符串表示全部回滚成功）
  auto rollback = [&]() -> std::string {
    std::string fails;
    if (has_exp && !mvs_camera_.setExposureTime(old_exp)) {
      fails += " exposure_time";
    }
    if (has_gain && !mvs_camera_.setGain(old_gain)) {
      fails += " gain";
    }
    if (has_fr && !mvs_camera_.setFrameRate(old_fr)) {
      fails += " frame_rate";
    }
    return fails;
  };

  // 通用失败处理：回滚 + 报告
  auto handle_failure = [&](const std::string & which) {
    std::string rb_fails = rollback();
    result.successful = false;
    if (rb_fails.empty()) {
      result.reason = "Failed to set " + which + ", rolled back successfully";
      RCLCPP_ERROR(get_logger(), "%s update failed, rolled back", which.c_str());
    } else {
      result.reason = "Failed to set " + which +
        ", AND rollback FAILED for:" + rb_fails +
        ". ROS parameters and camera hardware are now INCONSISTENT. "
        "Please re-issue the parameter set or restart the node.";
      RCLCPP_ERROR(get_logger(),
        "%s update failed, rollback FAILED for:%s. "
        "ROS parameters and hardware INCONSISTENT.",
        which.c_str(), rb_fails.c_str());
    }
    return result;
  };

  if (has_exp && !mvs_camera_.setExposureTime(new_exp)) {
    return handle_failure("exposure_time");
  }
  if (has_gain && !mvs_camera_.setGain(new_gain)) {
    return handle_failure("gain");
  }
  if (has_fr && !mvs_camera_.setFrameRate(new_fr)) {
    return handle_failure("frame_rate");
  }

  if (has_exp) RCLCPP_INFO(get_logger(), "Exposure time -> %.1f us", new_exp);
  if (has_gain) RCLCPP_INFO(get_logger(), "Gain -> %.2f dB", new_gain);
  if (has_fr) RCLCPP_INFO(get_logger(), "Frame rate -> %.2f Hz", new_fr);
  return result;
}

void CameraNode::timerCallback()
{
  if (reconnecting_.load()) {
    return;
  }

  // 一次调用完成"抓帧 + 转换 + 拷贝"，返回独立数据
  FrameData frame;
  GrabFormat fmt = (pixel_format_ == "bgr8") ? GrabFormat::Bgr : GrabFormat::RawBayer;

  if (!mvs_camera_.grabFrame(frame, fmt, 100)) {
    int fails = ++consecutive_failures_;

    if (fails >= 10 && !reconnect_needed_.load()) {
      RCLCPP_WARN(get_logger(),
        "Camera disconnected (%d consecutive failures), triggering reconnect", fails);
      reconnect_needed_ = true;
    }
    return;
  }

  consecutive_failures_ = 0;

  auto img = std::make_unique<sensor_msgs::msg::Image>();
  // 用 ROS 2 系统时间作为采集时间戳。
  // SDK 也提供 nDevTimeStampHigh/Low（设备 tick）和 nHostTimeStamp（主机时间戳），
  // 但单位未验证，暂不使用。
  img->header.stamp = this->now();
  img->header.frame_id = frame_id_;
  img->height = frame.height;
  img->width  = frame.width;
  img->encoding = frame.encoding;
  img->is_bigendian = 0;
  img->step = (fmt == GrabFormat::Bgr) ? (frame.width * 3) : frame.width;

  img->data = std::move(frame.data);

  publisher_->publish(std::move(img));
}

}  // namespace hikrobot_camera