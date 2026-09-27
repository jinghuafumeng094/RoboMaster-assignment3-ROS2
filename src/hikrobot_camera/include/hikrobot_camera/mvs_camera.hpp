#ifndef HIKROBOT_CAMERA__MVS_CAMERA_HPP_
#define HIKROBOT_CAMERA__MVS_CAMERA_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "MvCameraControl.h"

namespace hikrobot_camera
{

struct FrameInfo
{
  uint8_t * data = nullptr;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t data_len = 0;
  uint32_t frame_num = 0;
  // 设备时间戳：来自 SDK nDevTimeStampHigh/Low，单位未验证（通常是设备 tick 计数）
  uint64_t device_timestamp = 0;
  MvGvspPixelType pixel_type = PixelType_Gvsp_Undefined;
};

class MvsCamera
{
public:
  MvsCamera() = default;
  ~MvsCamera();

  MvsCamera(const MvsCamera &) = delete;
  MvsCamera & operator=(const MvsCamera &) = delete;

  bool listDevices(std::vector<MV_CC_DEVICE_INFO> & devices);
  bool openByIndex(unsigned int index);
  bool openBySerial(const std::string & serial);
  bool openByIp(const std::string & ip);

  bool startGrabbing();
  bool stopGrabbing();

  bool getFrame(FrameInfo & frame, unsigned int timeout_ms);
  void releaseFrame(FrameInfo & frame);

  bool convertToBgr(
    const FrameInfo & src,
    std::vector<uint8_t> & dst,
    uint32_t & dst_width,
    uint32_t & dst_height);

  void close();

  bool setExposureTime(double us);
  bool setGain(double db);
  bool setFrameRate(double fps);
  bool setTriggerMode(bool enable);

  bool getExposureTime(double & us);
  bool getGain(double & db);
  bool getFrameRate(double & fps);

  bool isOpen() const;

  // 最近一次 open 失败的原因，供上层打印
  std::string lastError() const;

private:
  bool openByDeviceInfo(const MV_CC_DEVICE_INFO & info);

  void * handle_ = nullptr;
  bool grabbing_ = false;
  mutable std::mutex mutex_;
  std::string last_error_;
};

}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__MVS_CAMERA_HPP_