/*
Copyright (c) 2025 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
/// @brief      Camera Nodelet
#ifndef TMC_PGR_CAMERA_CAMERA_NODELET_HPP_
#define TMC_PGR_CAMERA_CAMERA_NODELET_HPP_

#include <memory>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include "tmc_vision_msgs/srv/control_camera.hpp"

namespace tmc_pgr_camera {

class ICameraSystemPluginBase;
class CameraImagesPublisher;
class IPointGreyCameraSystemSetting;

/// @brief Camera Nodelet class
class CameraNodelet : public rclcpp::Node {
  CameraNodelet(const CameraNodelet&) = delete;
  CameraNodelet& operator=(const CameraNodelet&) = delete;

 public:
  /// Constructor
  explicit CameraNodelet(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  /// Destructor
  virtual ~CameraNodelet();

  /// Initialization function
  void onInit();

  /// Capture thread
  void CaptureThread();

  /// Receive capture start/stop commands from HMI
  bool ControlCameraCallback(const std::shared_ptr<tmc_vision_msgs::srv::ControlCamera::Request> req,
                             std::shared_ptr<tmc_vision_msgs::srv::ControlCamera::Response> res);

  /// Convert camera settings from parameter to YAML and wait for setting changes
  void ChangeCameraProperties();

  /// Callback when there is a change in the parameter
  rcl_interfaces::msg::SetParametersResult SetParameterCallback(const std::vector<rclcpp::Parameter>& params);

 private:
  /// Object to load camera plugin
  std::shared_ptr<pluginlib::ClassLoader<ICameraSystemPluginBase> > camera_loader_;
  /// Camera object
  std::shared_ptr<ICameraSystemPluginBase> camera_;
  /// Camera image distributor
  std::shared_ptr<CameraImagesPublisher> camera_images_publisher_;
  /// Service that provides control for starting/stopping capture
  rclcpp::Service<tmc_vision_msgs::srv::ControlCamera>::SharedPtr control_camera_service_;
  /// Capture thread
  std::shared_ptr<std::thread> capture_thread_;
  /// Thread stop flag
  bool can_stop_capture_thread_;
  /// Mutex
  std::shared_mutex access_;
  /// Reference to the camera configuration interface
  std::shared_ptr<IPointGreyCameraSystemSetting> camera_setting_;
  /// dynamic_reconfigure server
  // std::shared_ptr<dynamic_reconfigure::Server <CameraPropertyConfig> > dynamic_reconfigure_server_;
  /// Handle for monitoring changes in the parameter
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr set_parameters_handle_;
  /// Change camera settings received in param via YAML
  YAML::Node camera_properties_;
  /// Determine that there was an update in the parameter
  bool property_changed_;
};

}  // end of namespace tmc_pgr_camera

#endif  // TMC_PGR_CAMERA_CAMERA_NODELET_HPP_
