#ifndef HIKROBOT_CAMERA__MVS_CAMERA_HPP_
#define HIKROBOT_CAMERA__MVS_CAMERA_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "MvCameraControl.h"

namespace hikrobot_camera
{

// 一帧图像的元数据
struct FrameInfo
{
  uint8_t * data = nullptr;         // 数据指针（来自 SDK 缓冲区）
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t data_len = 0;
  uint32_t frame_num = 0;
  uint64_t timestamp_ns = 0;        // 设备时间戳（纳秒）
  MvGvspPixelType pixel_type = PixelType_Gvsp_Undefined;
};

class MvsCamera
{
public:
  MvsCamera() = default;
  ~MvsCamera();

  MvsCamera(const MvsCamera &) = delete;
  MvsCamera & operator=(const MvsCamera &) = delete;

  // 枚举设备（复制一份设备信息，避免直接持有 SDK 内部指针）
  bool listDevices(std::vector<MV_CC_DEVICE_INFO> & devices);

  // 打开设备
  bool openByIndex(unsigned int index);
  bool openBySerial(const std::string & serial);
  bool openByIp(const std::string & ip);

  // 取流
  bool startGrabbing();
  bool stopGrabbing();

  // 抓一帧（阻塞），成功返回 true
  bool getFrame(FrameInfo & frame, unsigned int timeout_ms);
  void releaseFrame(FrameInfo & frame);

  // 关闭
  void close();

  // 参数
  bool setExposureTime(double us);
  bool setGain(double db);
  bool setFrameRate(double fps);
  bool setPixelFormat(const std::string & format);
  bool setTriggerMode(bool enable);

  bool getExposureTime(double & us);
  bool getGain(double & db);
  bool getFrameRate(double & fps);

  bool isOpen() const { return handle_ != nullptr; }

private:
  void * handle_ = nullptr;   // MV_CC 句柄
  bool grabbing_ = false;
};

}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__MVS_CAMERA_HPP_