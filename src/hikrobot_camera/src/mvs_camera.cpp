#include "hikrobot_camera/mvs_camera.hpp"

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
  ensureSdkInitialized();

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

bool MvsCamera::convertToBgr(
  const FrameInfo & src,
  std::vector<uint8_t> & dst,
  uint32_t & dst_width,
  uint32_t & dst_height)
{
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

bool MvsCamera::setExposureTime(double us)
{
  if (handle_ == nullptr) {
    return false;
  }

  int nRet = MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
  if (nRet != MV_OK) {
    return false;
  }

  nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(us));
  return nRet == MV_OK;
}

bool MvsCamera::setGain(double db)
{
  if (handle_ == nullptr) {
    return false;
  }

  int nRet = MV_CC_SetEnumValue(handle_, "GainAuto", 0);
  if (nRet != MV_OK) {
    return false;
  }

  nRet = MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(db));
  return nRet == MV_OK;
}

bool MvsCamera::setFrameRate(double fps)
{
  if (handle_ == nullptr) {
    return false;
  }

  int nRet = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
  if (nRet != MV_OK) {
    return false;
  }

  nRet = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", static_cast<float>(fps));
  return nRet == MV_OK;
}

bool MvsCamera::setTriggerMode(bool enable)
{
  if (handle_ == nullptr) {
    return false;
  }

  int nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", enable ? 1 : 0);
  return nRet == MV_OK;
}

bool MvsCamera::getExposureTime(double & us)
{
  if (handle_ == nullptr) {
    return false;
  }

  MVCC_FLOATVALUE stFloatValue;
  std::memset(&stFloatValue, 0, sizeof(MVCC_FLOATVALUE));

  int nRet = MV_CC_GetFloatValue(handle_, "ExposureTime", &stFloatValue);
  if (nRet != MV_OK) {
    return false;
  }
  us = stFloatValue.fCurValue;
  return true;
}

bool MvsCamera::getGain(double & db)
{
  if (handle_ == nullptr) {
    return false;
  }

  MVCC_FLOATVALUE stFloatValue;
  std::memset(&stFloatValue, 0, sizeof(MVCC_FLOATVALUE));

  int nRet = MV_CC_GetFloatValue(handle_, "Gain", &stFloatValue);
  if (nRet != MV_OK) {
    return false;
  }
  db = stFloatValue.fCurValue;
  return true;
}

bool MvsCamera::getFrameRate(double & fps)
{
  if (handle_ == nullptr) {
    return false;
  }

  MVCC_FLOATVALUE stFloatValue;
  std::memset(&stFloatValue, 0, sizeof(MVCC_FLOATVALUE));

  int nRet = MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &stFloatValue);
  if (nRet != MV_OK) {
    return false;
  }
  fps = stFloatValue.fCurValue;
  return true;
}

}  // namespace hikrobot_camera