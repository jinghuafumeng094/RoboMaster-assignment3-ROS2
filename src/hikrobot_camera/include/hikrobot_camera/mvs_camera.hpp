#ifndef HIKROBOT_CAMERA__MVS_CAMERA_HPP_
#define HIKROBOT_CAMERA__MVS_CAMERA_HPP_

#include <cstdint>
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
  uint64_t timestamp_ns = 0;
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

  // 参数设置 / 读取
  bool setExposureTime(double us);
  bool setGain(double db);
  bool setFrameRate(double fps);
  bool setTriggerMode(bool enable);

  bool getExposureTime(double & us);
  bool getGain(double & db);
  bool getFrameRate(double & fps);

  bool isOpen() const { return handle_ != nullptr; }

private:
  void * handle_ = nullptr;
  bool grabbing_ = false;
};

}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__MVS_CAMERA_HPP_