#ifndef HIKROBOT_CAMERA__MVS_CAMERA_HPP_
#define HIKROBOT_CAMERA__MVS_CAMERA_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "MvCameraControl.h"

namespace hikrobot_camera
{

// 目标像素格式
enum class GrabFormat
{
  Bgr,      
  RawBayer  
};

struct FrameData
{
  std::vector<uint8_t> data;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t frame_num = 0;
  uint64_t device_timestamp = 0;
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

  // 原子化抓帧：一次加锁完成 GetImageBuffer → 转换/拷贝 → FreeImageBuffer
  // 返回后 out.data 是独立拷贝，SDK 缓冲区已释放，外部拿不到裸指针
  bool grabFrame(FrameData & out, GrabFormat format, unsigned int timeout_ms);

  void close();

  bool setExposureTime(double us);
  bool setGain(double db);
  bool setFrameRate(double fps);
  bool setTriggerMode(bool enable);

  bool getExposureTime(double & us);
  bool getGain(double & db);
  bool getFrameRate(double & fps);

  bool isOpen() const;

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