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
/// @brief Camera system plugin for testing
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <pluginlib/class_list_macros.hpp>
#include "tmc_pgr_camera/i_camera_system_plugin_base.hpp"
#include "tmc_pgr_camera/i_point_grey_camera_system_setting.hpp"

namespace tmc_pgr_camera {

/// @brief Camera system for testing
class DummyCameraSystemPlugin : public ICameraSystemPluginBase {
 public:
  /// Constructor
  DummyCameraSystemPlugin() : is_opened(false), is_capturing(false) {}

  /// Plugin initialization function
  virtual void Initialize(std::shared_ptr<IPointGreyCameraSystemSetting>& camera_setting) {
    camera_system_settings_ = camera_setting;
    Open();
  }

  /// Start the camera system
  virtual void Open() {
    is_opened = true;
  }
  /// Shutdown the camera system
  virtual void Close() {
    is_opened = false;
    StopCapture();
  }

  /// Start capturing
  virtual void StartCapture() { is_capturing = true; }
  /// Stop capturing
  virtual void StopCapture() { is_capturing = false; }

  /// Get captured image
  virtual std::optional<std::vector<ImagePtr> > GrabImage() {
    ImagePtr image(new Image());
    image->image = cv::Mat::eye(5, 5, CV_8U);
    image->time = std::chrono::system_clock::now();
    std::vector<ImagePtr> images;
    const std::vector<uint32_t> serial_numbers = camera_system_settings_->GetSerialNumbers();
    for (int32_t i = 0; i < serial_numbers.size(); ++i) {
      images.push_back(image);
    }
    return images;
  }

  /// Check if the camera system is running
  virtual bool IsOpened() const { return is_opened; }

  /// Check if capturing is ongoing
  virtual bool IsCapturing() const { return is_capturing; }

  /// Configure the camera settings
  virtual void SetSettings(const YAML::Node& settings) {
    // It's not possible in ROS 2 to begin with
    // ros::param::set("test_dynamic_reconfigure_callback_setting", settings);
  }

 private:
  /// Whether the camera system is active
  bool is_opened;

  /// Whether capturing is in progress
  bool is_capturing;

  /// Settings load object
  std::shared_ptr<IPointGreyCameraSystemSetting> camera_system_settings_;
};

}  // end of namespace tmc_pgr_camera

PLUGINLIB_EXPORT_CLASS(tmc_pgr_camera::DummyCameraSystemPlugin, tmc_pgr_camera::ICameraSystemPluginBase);
