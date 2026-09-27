#ifndef HIKROBOT_CAMERA__CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA__CAMERA_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "hikrobot_camera/mvs_camera.hpp"

namespace hikrobot_camera
{

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  private:
    MvsCamera mvs_camera_;
  // TODO(student): Design the interfaces and resource ownership required by
  // your implementation. No SDK handles or camera operations are provided.
};

}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__CAMERA_NODE_HPP_
