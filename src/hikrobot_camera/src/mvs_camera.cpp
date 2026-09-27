#include "hikrobot_camera/mvs_camera.hpp"

#include <cstring>

namespace hikrobot_camera
{

MvsCamera::~MvsCamera()
{
  close();
}

void MvsCamera::close()
{
  // TODO: 关闭设备、销毁句柄，后面实现
}

bool MvsCamera::listDevices(std::vector<MV_CC_DEVICE_INFO> & devices)
{
  devices.clear();

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

// 其余方法先占位，保证编译通过
bool MvsCamera::openByIndex(unsigned int) { return false; }
bool MvsCamera::openBySerial(const std::string &) { return false; }
bool MvsCamera::openByIp(const std::string &) { return false; }

bool MvsCamera::startGrabbing() { return false; }
bool MvsCamera::stopGrabbing() { return false; }

bool MvsCamera::getFrame(FrameInfo &, unsigned int) { return false; }
void MvsCamera::releaseFrame(FrameInfo &) {}

bool MvsCamera::setExposureTime(double) { return false; }
bool MvsCamera::setGain(double) { return false; }
bool MvsCamera::setFrameRate(double) { return false; }
bool MvsCamera::setPixelFormat(const std::string &) { return false; }
bool MvsCamera::setTriggerMode(bool) { return false; }

bool MvsCamera::getExposureTime(double &) { return false; }
bool MvsCamera::getGain(double &) { return false; }
bool MvsCamera::getFrameRate(double &) { return false; }

}  // namespace hikrobot_camera