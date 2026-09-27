#include "hikrobot_camera/mvs_camera.hpp"

#include <cstring>
#include <mutex>

namespace hikrobot_camera
{

namespace
{
void ensureSdkInitialized()
{
  static std::once_flag flag;
  std::call_once(flag, []() {
    int ret = MV_CC_Initialize();
    if (ret != MV_OK) {
      // 初始化失败通常在 open 时由具体 API 报错体现
    }
  });
}
}  // namespace

MvsCamera::~MvsCamera()
{
  close();
}

bool MvsCamera::listDevices(std::vector<MV_CC_DEVICE_INFO> & devices)
{
  devices.clear();
  ensureSdkInitialized();

  MV_CC_DEVICE_INFO_LIST stDeviceList;
  std::memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (nRet != MV_OK) {
    return false;
  }

  for (unsigned int i = 0; i < stDeviceList.nDeviceNum; ++i) {
    if (stDeviceList.pDeviceInfo[i] != nullptr) {
      devices.push_back(*stDeviceList.pDeviceInfo[i]);
    }
  }
  return true;
}

bool MvsCamera::openByIndex(unsigned int index)
{
  ensureSdkInitialized();

  std::vector<MV_CC_DEVICE_INFO> devices;
  if (!listDevices(devices)) {
    return false;
  }
  if (index >= devices.size()) {
    return false;
  }

  int nRet = MV_CC_CreateHandle(&handle_, &devices[index]);
  if (nRet != MV_OK) {
    handle_ = nullptr;
    return false;
  }

  nRet = MV_CC_OpenDevice(handle_);
  if (nRet != MV_OK) {
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return false;
  }
  return true;
}

bool MvsCamera::startGrabbing()
{
  if (handle_ == nullptr) {
    return false;
  }

  int nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
  if (nRet != MV_OK) {
    return false;
  }

  nRet = MV_CC_StartGrabbing(handle_);
  if (nRet != MV_OK) {
    return false;
  }

  grabbing_ = true;
  return true;
}

bool MvsCamera::stopGrabbing()
{
  if (handle_ == nullptr || !grabbing_) {
    return true;
  }

  int nRet = MV_CC_StopGrabbing(handle_);
  grabbing_ = false;
  return nRet == MV_OK;
}

bool MvsCamera::getFrame(FrameInfo & frame, unsigned int timeout_ms)
{
  if (handle_ == nullptr) {
    return false;
  }

  MV_FRAME_OUT stImageInfo;
  std::memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));

  int nRet = MV_CC_GetImageBuffer(handle_, &stImageInfo, timeout_ms);
  if (nRet != MV_OK) {
    return false;
  }

  frame.data = stImageInfo.pBufAddr;
  frame.width = stImageInfo.stFrameInfo.nExtendWidth;
  frame.height = stImageInfo.stFrameInfo.nExtendHeight;
  frame.data_len = stImageInfo.stFrameInfo.nFrameLen;
  frame.frame_num = stImageInfo.stFrameInfo.nFrameNum;
  frame.pixel_type = stImageInfo.stFrameInfo.enPixelType;

  // 设备时间戳（DevTimeStamp 可能是高精度时间戳，单位依赖于设备）
  frame.timestamp_ns = stImageInfo.stFrameInfo.nDevTimeStampHigh;
  frame.timestamp_ns = (frame.timestamp_ns << 32) | stImageInfo.stFrameInfo.nDevTimeStampLow;

  return true;
}

void MvsCamera::releaseFrame(FrameInfo & frame)
{
  if (handle_ == nullptr || frame.data == nullptr) {
    return;
  }

  MV_FRAME_OUT stImageInfo;
  std::memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));
  stImageInfo.pBufAddr = frame.data;
  MV_CC_FreeImageBuffer(handle_, &stImageInfo);

  frame.data = nullptr;
}

void MvsCamera::close()
{
  if (handle_ == nullptr) {
    return;
  }

  if (grabbing_) {
    MV_CC_StopGrabbing(handle_);
    grabbing_ = false;
  }

  MV_CC_CloseDevice(handle_);
  MV_CC_DestroyHandle(handle_);
  handle_ = nullptr;
}

// 参数相关方法暂时占位
bool MvsCamera::openBySerial(const std::string &) { return false; }
bool MvsCamera::openByIp(const std::string &) { return false; }

bool MvsCamera::setExposureTime(double) { return false; }
bool MvsCamera::setGain(double) { return false; }
bool MvsCamera::setFrameRate(double) { return false; }
bool MvsCamera::setPixelFormat(const std::string &) { return false; }
bool MvsCamera::setTriggerMode(bool) { return false; }

bool MvsCamera::getExposureTime(double &) { return false; }
bool MvsCamera::getGain(double &) { return false; }
bool MvsCamera::getFrameRate(double &) { return false; }

}  // namespace hikrobot_camera