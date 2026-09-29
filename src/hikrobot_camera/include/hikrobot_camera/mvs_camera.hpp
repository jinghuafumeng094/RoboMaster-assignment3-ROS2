#ifndef HIKROBOT_CAMERA__MVS_CAMERA_HPP_
#define HIKROBOT_CAMERA__MVS_CAMERA_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "MvCameraControl.h"

namespace hikrobot_camera
{

// 一帧抓取结果。data 由调用方传入的 vector 承载，内部复用容量。
struct FrameData
{
  std::vector<uint8_t> data;   
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t frame_num = 0;
  MvGvspPixelType src_pixel_type = PixelType_Gvsp_Undefined;
  std::string encoding;
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

  // 抓帧：一次加锁，内部完成 GetImageBuffer → 拷贝到 out.data → FreeImageBuffer
  bool grabFrame(FrameData & out, unsigned int timeout_ms);

  void close();

  bool setExposureTime(double us);
  bool setGain(double db);
  bool setFrameRate(double fps);
  bool setTriggerMode(bool enable);
  // 只支持 "bgr8" 和 "bayer_rggb8"，真的设置相机 PixelFormat 节点
  bool setPixelFormat(const std::string & fmt);

  bool getExposureTime(double & us);
  bool getGain(double & db);
  bool getFrameRate(double & fps);

  bool isOpen() const;
  std::string lastError() const;

private:
  bool openByDeviceInfo(const MV_CC_DEVICE_INFO & info);
  static std::string pixelTypeToEncoding(MvGvspPixelType type);

  void * handle_ = nullptr;
  bool grabbing_ = false;
  mutable std::mutex mutex_;
  std::string last_error_;
};

}  // namespace hikrobot_camera

#endif