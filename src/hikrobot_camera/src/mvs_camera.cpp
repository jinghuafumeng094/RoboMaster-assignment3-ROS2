#include "hikrobot_camera/mvs_camera.hpp"

#include <cmath>
#include <cstdio>
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
    MV_CC_Initialize();
  });
}
}  // namespace

MvsCamera::~MvsCamera()
{
  close();
}

bool MvsCamera::isOpen() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return handle_ != nullptr;
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

bool MvsCamera::openByDeviceInfo(const MV_CC_DEVICE_INFO & info)
{
  ensureSdkInitialized();

  int nRet = MV_CC_CreateHandle(&handle_, &info);
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

bool MvsCamera::openByIndex(unsigned int index)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ != nullptr) {
    return false;
  }

  std::vector<MV_CC_DEVICE_INFO> devices;
  if (!listDevices(devices)) {
    return false;
  }
  if (index >= devices.size()) {
    return false;
  }
  return openByDeviceInfo(devices[index]);
}

bool MvsCamera::openBySerial(const std::string & serial)
{
  if (serial.empty()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ != nullptr) {
    return false;
  }

  std::vector<MV_CC_DEVICE_INFO> devices;
  if (!listDevices(devices)) {
    return false;
  }

  const MV_CC_DEVICE_INFO * matched = nullptr;
  for (const auto & info : devices) {
    std::string dev_serial;
    if (info.nTLayerType == MV_USB_DEVICE) {
      dev_serial = reinterpret_cast<const char *>(info.SpecialInfo.stUsb3VInfo.chSerialNumber);
    } else if (info.nTLayerType == MV_GIGE_DEVICE) {
      dev_serial = reinterpret_cast<const char *>(info.SpecialInfo.stGigEInfo.chSerialNumber);
    } else {
      continue;
    }

    if (serial == dev_serial) {
      matched = &info;
      break;
    }
  }

  if (matched == nullptr) {
    return false;
  }
  return openByDeviceInfo(*matched);
}

bool MvsCamera::openByIp(const std::string & ip)
{
  if (ip.empty()) {
    return false;
  }

  uint32_t target_ip = 0;
  {
    unsigned int a = 0, b = 0, c = 0, d = 0;
    if (std::sscanf(ip.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) != 4) {
      return false;
    }
    if (a > 255 || b > 255 || c > 255 || d > 255) {
      return false;
    }
    target_ip = (a << 24) | (b << 16) | (c << 8) | d;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ != nullptr) {
    return false;
  }

  std::vector<MV_CC_DEVICE_INFO> devices;
  if (!listDevices(devices)) {
    return false;
  }

  const MV_CC_DEVICE_INFO * matched = nullptr;
  for (const auto & info : devices) {
    if (info.nTLayerType == MV_GIGE_DEVICE) {
      if (info.SpecialInfo.stGigEInfo.nCurrentIp == target_ip) {
        matched = &info;
        break;
      }
    }
  }

  if (matched == nullptr) {
    return false;
  }
  return openByDeviceInfo(*matched);
}

bool MvsCamera::startGrabbing()
{
  std::lock_guard<std::mutex> lock(mutex_);
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
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr || !grabbing_) {
    return true;
  }

  int nRet = MV_CC_StopGrabbing(handle_);
  grabbing_ = false;
  return nRet == MV_OK;
}

bool MvsCamera::getFrame(FrameInfo & frame, unsigned int timeout_ms)
{
  std::lock_guard<std::mutex> lock(mutex_);
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

  frame.timestamp_ns = stImageInfo.stFrameInfo.nDevTimeStampHigh;
  frame.timestamp_ns = (frame.timestamp_ns << 32) | stImageInfo.stFrameInfo.nDevTimeStampLow;

  return true;
}

void MvsCamera::releaseFrame(FrameInfo & frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr || frame.data == nullptr) {
    return;
  }

  MV_FRAME_OUT stImageInfo;
  std::memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));
  stImageInfo.pBufAddr = frame.data;
  MV_CC_FreeImageBuffer(handle_, &stImageInfo);

  frame.data = nullptr;
}

bool MvsCamera::convertToBgr(
  const FrameInfo & src,
  std::vector<uint8_t> & dst,
  uint32_t & dst_width,
  uint32_t & dst_height)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr || src.data == nullptr) {
    return false;
  }

  dst_width = src.width;
  dst_height = src.height;

  size_t needed = static_cast<size_t>(src.width) * src.height * 3;
  if (dst.size() < needed) {
    dst.resize(needed);
  }

  MV_CC_PIXEL_CONVERT_PARAM stConvertParam;
  std::memset(&stConvertParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));

  stConvertParam.nWidth         = src.width;
  stConvertParam.nHeight        = src.height;
  stConvertParam.pSrcData       = src.data;
  stConvertParam.nSrcDataLen    = src.data_len;
  stConvertParam.enSrcPixelType = src.pixel_type;
  stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
  stConvertParam.pDstBuffer     = dst.data();
  stConvertParam.nDstBufferSize = static_cast<unsigned int>(needed);

  int nRet = MV_CC_ConvertPixelType(handle_, &stConvertParam);
  return nRet == MV_OK;
}

void MvsCamera::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
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

// ---------- 参数设置：写完后读回，不一致则视为失败 ----------

bool MvsCamera::setExposureTime(double us)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr) {
    return false;
  }

  if (MV_CC_SetEnumValue(handle_, "ExposureAuto", 0) != MV_OK) {
    return false;
  }
  if (MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(us)) != MV_OK) {
    return false;
  }

  MVCC_FLOATVALUE stFloatValue;
  std::memset(&stFloatValue, 0, sizeof(MVCC_FLOATVALUE));
  if (MV_CC_GetFloatValue(handle_, "ExposureTime", &stFloatValue) != MV_OK) {
    return false;
  }
  return std::abs(stFloatValue.fCurValue - us) <= 1.0;
}

bool MvsCamera::setGain(double db)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr) {
    return false;
  }

  if (MV_CC_SetEnumValue(handle_, "GainAuto", 0) != MV_OK) {
    return false;
  }
  if (MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(db)) != MV_OK) {
    return false;
  }

  MVCC_FLOATVALUE stFloatValue;
  std::memset(&stFloatValue, 0, sizeof(MVCC_FLOATVALUE));
  if (MV_CC_GetFloatValue(handle_, "Gain", &stFloatValue) != MV_OK) {
    return false;
  }
  return std::abs(stFloatValue.fCurValue - db) <= 0.1;
}

bool MvsCamera::setFrameRate(double fps)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr) {
    return false;
  }

  if (MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true) != MV_OK) {
    return false;
  }
  if (MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", static_cast<float>(fps)) != MV_OK) {
    return false;
  }

  MVCC_FLOATVALUE stFloatValue;
  std::memset(&stFloatValue, 0, sizeof(MVCC_FLOATVALUE));
  if (MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &stFloatValue) != MV_OK) {
    return false;
  }
  return std::abs(stFloatValue.fCurValue - fps) <= 0.1;
}

bool MvsCamera::setTriggerMode(bool enable)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr) {
    return false;
  }
  return MV_CC_SetEnumValue(handle_, "TriggerMode", enable ? 1 : 0) == MV_OK;
}

bool MvsCamera::getExposureTime(double & us)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr) {
    return false;
  }
  MVCC_FLOATVALUE stFloatValue;
  std::memset(&stFloatValue, 0, sizeof(MVCC_FLOATVALUE));
  if (MV_CC_GetFloatValue(handle_, "ExposureTime", &stFloatValue) != MV_OK) {
    return false;
  }
  us = stFloatValue.fCurValue;
  return true;
}

bool MvsCamera::getGain(double & db)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr) {
    return false;
  }
  MVCC_FLOATVALUE stFloatValue;
  std::memset(&stFloatValue, 0, sizeof(MVCC_FLOATVALUE));
  if (MV_CC_GetFloatValue(handle_, "Gain", &stFloatValue) != MV_OK) {
    return false;
  }
  db = stFloatValue.fCurValue;
  return true;
}

bool MvsCamera::getFrameRate(double & fps)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr) {
    return false;
  }
  MVCC_FLOATVALUE stFloatValue;
  std::memset(&stFloatValue, 0, sizeof(MVCC_FLOATVALUE));
  if (MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &stFloatValue) != MV_OK) {
    return false;
  }
  fps = stFloatValue.fCurValue;
  return true;
}

}  // namespace hikrobot_camera