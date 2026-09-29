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

std::string MvsCamera::lastError() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

bool MvsCamera::isOpen() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return handle_ != nullptr;
}

std::string MvsCamera::pixelTypeToEncoding(MvGvspPixelType type)
{
  switch (type) {
    case PixelType_Gvsp_BGR8_Packed:  return "bgr8";
    case PixelType_Gvsp_BayerRG8:     return "bayer_rggb8";
    default:                          return "unknown";
  }
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
    last_error_ = "MV_CC_CreateHandle failed: 0x" + std::to_string(nRet);
    return false;
  }

  nRet = MV_CC_OpenDevice(handle_);
  if (nRet != MV_OK) {
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    last_error_ = "MV_CC_OpenDevice failed: 0x" + std::to_string(nRet);
    return false;
  }
  return true;
}

bool MvsCamera::openByIndex(unsigned int index)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ != nullptr) {
    last_error_ = "Camera already open";
    return false;
  }

  std::vector<MV_CC_DEVICE_INFO> devices;
  if (!listDevices(devices)) {
    last_error_ = "MV_CC_EnumDevices failed";
    return false;
  }
  if (index >= devices.size()) {
    last_error_ = "Index " + std::to_string(index) +
                  " out of range, found " + std::to_string(devices.size()) + " device(s)";
    return false;
  }
  last_error_.clear();
  return openByDeviceInfo(devices[index]);
}

bool MvsCamera::openBySerial(const std::string & serial)
{
  if (serial.empty()) {
    last_error_ = "Serial is empty";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ != nullptr) {
    last_error_ = "Camera already open";
    return false;
  }

  std::vector<MV_CC_DEVICE_INFO> devices;
  if (!listDevices(devices)) {
    last_error_ = "MV_CC_EnumDevices failed";
    return false;
  }

  const MV_CC_DEVICE_INFO * matched = nullptr;
  int match_count = 0;
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
      ++match_count;
    }
  }

  if (match_count == 0) {
    last_error_ = "No camera with serial '" + serial + "'";
    return false;
  }
  if (match_count > 1) {
    last_error_ = "Ambiguous: " + std::to_string(match_count) +
                  " cameras have serial '" + serial + "'";
    return false;
  }
  last_error_.clear();
  return openByDeviceInfo(*matched);
}

bool MvsCamera::openByIp(const std::string & ip)
{
  if (ip.empty()) {
    last_error_ = "IP is empty";
    return false;
  }

  uint32_t target_ip = 0;
  {
    unsigned int a = 0, b = 0, c = 0, d = 0;
    if (std::sscanf(ip.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) != 4) {
      last_error_ = "Invalid IP format: " + ip;
      return false;
    }
    if (a > 255 || b > 255 || c > 255 || d > 255) {
      last_error_ = "Invalid IP range: " + ip;
      return false;
    }
    target_ip = (a << 24) | (b << 16) | (c << 8) | d;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ != nullptr) {
    last_error_ = "Camera already open";
    return false;
  }

  std::vector<MV_CC_DEVICE_INFO> devices;
  if (!listDevices(devices)) {
    last_error_ = "MV_CC_EnumDevices failed";
    return false;
  }

  const MV_CC_DEVICE_INFO * matched = nullptr;
  int match_count = 0;
  for (const auto & info : devices) {
    if (info.nTLayerType == MV_GIGE_DEVICE) {
      if (info.SpecialInfo.stGigEInfo.nCurrentIp == target_ip) {
        matched = &info;
        ++match_count;
      }
    }
  }

  if (match_count == 0) {
    last_error_ = "No GigE camera with IP '" + ip + "'";
    return false;
  }
  if (match_count > 1) {
    last_error_ = "Ambiguous: " + std::to_string(match_count) +
                  " cameras have IP '" + ip + "'";
    return false;
  }
  last_error_.clear();
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

bool MvsCamera::grabFrame(FrameData & out, unsigned int timeout_ms)
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

  out.width          = stImageInfo.stFrameInfo.nExtendWidth;
  out.height         = stImageInfo.stFrameInfo.nExtendHeight;
  out.frame_num      = stImageInfo.stFrameInfo.nFrameNum;
  out.src_pixel_type = stImageInfo.stFrameInfo.enPixelType;

  // 复用 out.data 容量：size 和 needed 相等时不分配
  const size_t needed = stImageInfo.stFrameInfo.nFrameLen;
  if (out.data.size() != needed) {
    out.data.resize(needed);
  }
  std::memcpy(out.data.data(), stImageInfo.pBufAddr, needed);

  out.encoding = pixelTypeToEncoding(stImageInfo.stFrameInfo.enPixelType);

  MV_CC_FreeImageBuffer(handle_, &stImageInfo);

  return true;
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

// ---------------- 参数 ----------------

bool MvsCamera::setPixelFormat(const std::string & fmt)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (handle_ == nullptr) {
    last_error_ = "Camera not open";
    return false;
  }

  // 目标格式
  MvGvspPixelType target;
  if (fmt == "bgr8") {
    target = PixelType_Gvsp_BGR8_Packed;
  } else if (fmt == "bayer_rggb8") {
    target = PixelType_Gvsp_BayerRG8;
  } else {
    last_error_ = "Unsupported pixel format: " + fmt +
                  " (only 'bgr8' or 'bayer_rggb8')";
    return false;
  }

  // 洞二修复：设之前先读原格式，用于失败回滚
  MvGvspPixelType original = PixelType_Gvsp_Undefined;
  {
    MVCC_ENUMVALUE stCur;
    std::memset(&stCur, 0, sizeof(MVCC_ENUMVALUE));
    if (MV_CC_GetEnumValue(handle_, "PixelFormat", &stCur) == MV_OK) {
      original = static_cast<MvGvspPixelType>(stCur.nCurValue);
    }
  }

  // 取流中不能改 PixelFormat，先停
  bool was_grabbing = grabbing_;
  if (was_grabbing) {
    MV_CC_StopGrabbing(handle_);
    grabbing_ = false;
  }

  // 洞三修复：恢复取流的辅助 lambda，检查返回值
  auto restore_grabbing = [&]() -> bool {
    if (!was_grabbing) {
      return true;
    }
    int r = MV_CC_StartGrabbing(handle_);
    if (r == MV_OK) {
      grabbing_ = true;
      return true;
    }
    last_error_ += " (restart grabbing failed: 0x" + std::to_string(r) + ")";
    return false;
  };

  // 回滚辅助 lambda
  auto rollback = [&]() {
    if (original != PixelType_Gvsp_Undefined && original != target) {
      MV_CC_SetEnumValue(handle_, "PixelFormat", original);
    }
  };

  // 设置新格式
  int nRet = MV_CC_SetEnumValue(handle_, "PixelFormat", target);
  if (nRet != MV_OK) {
    last_error_ = "MV_CC_SetEnumValue(PixelFormat) failed: 0x" + std::to_string(nRet);
    restore_grabbing();
    return false;
  }

  // 洞一修复：读回失败 = 判失败
  MVCC_ENUMVALUE stEnumValue;
  std::memset(&stEnumValue, 0, sizeof(MVCC_ENUMVALUE));
  nRet = MV_CC_GetEnumValue(handle_, "PixelFormat", &stEnumValue);

  bool readback_ok = false;
  if (nRet != MV_OK) {
    last_error_ = "MV_CC_GetEnumValue(PixelFormat) failed: 0x" + std::to_string(nRet);
  } else if (stEnumValue.nCurValue != static_cast<unsigned int>(target)) {
    last_error_ = "PixelFormat readback mismatch: expected " +
                  std::to_string(target) + ", got " +
                  std::to_string(stEnumValue.nCurValue);
  } else {
    readback_ok = true;
  }

  if (!readback_ok) {
    // 洞二修复：mismatch/读回失败时回滚
    rollback();
    restore_grabbing();
    return false;
  }

  // 成功，恢复取流（洞三修复：检查返回值）
  if (!restore_grabbing()) {
    return false;
  }

  last_error_.clear();
  return true;
}

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