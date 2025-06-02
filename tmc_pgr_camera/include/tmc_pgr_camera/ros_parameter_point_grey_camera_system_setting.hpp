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
/// @brief      Retrieve the Point Grey camera system configuration from the parameter server
#ifndef TMC_PGR_CAMERA_ROS_PARAMETER_POINT_GREY_CAMERA_SYSTEM_SETTING_HPP_
#define TMC_PGR_CAMERA_ROS_PARAMETER_POINT_GREY_CAMERA_SYSTEM_SETTING_HPP_
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>

#include "tmc_pgr_camera/i_point_grey_camera_system_setting.hpp"

namespace tmc_pgr_camera {

/// @brief Class to retrieve the Point Grey camera system configuration from the ROS parameter server
class RosParameterPointGreyCameraSystemSetting : public IPointGreyCameraSystemSetting {
 public:
  /// Constructor
  explicit RosParameterPointGreyCameraSystemSetting(rclcpp::Node::SharedPtr node_handle);

  /// Get an array of serial numbers of cameras used in the camera system
  virtual std::vector<uint32_t> GetSerialNumbers();

  /// Load an array of camera properties
  virtual std::vector<FlyCapture2::Property> GetProperties();

  /// Get the frame rate
  virtual std::optional<std::pair<FlyCapture2::FrameRate, float> >
      GetFrameRate(const FlyCapture2::VideoMode video_mode);

  /// Get the video mode
  virtual std::optional<FlyCapture2::VideoMode> GetVideoMode();

  /// Get the Format7 settings
  virtual std::optional<FlyCapture2::Format7ImageSettings> GetFormat7Setting();

  /// Get the software demosaicing settings
  virtual std::optional<FlyCapture2::ColorProcessingAlgorithm> GetSoftDemosaicing();

  /// Check if the software trigger is enabled
  virtual bool IsSoftwareTriggerEnabled();

  /// Check if the self-trigger is enabled
  virtual bool IsSelfTriggerEnabled();

  /// Get the self-trigger settings
  virtual std::optional<SelfTriggerSettings> GetSelfTriggerSettings();

  /// Get the trigger mode settings
  virtual std::optional<FlyCapture2::TriggerMode> GetTriggerMode();

  /// Create the trigger mode settings
  virtual void UpdateTriggerMode(int trigger_mode_mode, bool trigger_mode_on_off, int trigger_mode_polarity);

  /// Get the trigger delay settings
  virtual std::optional<FlyCapture2::TriggerDelay> GetTriggerDelay();

  /// Get the settings of the image type (monochrome, color)
  virtual std::optional<ImageType> GetImageType();

  /// Get the 3.3V output settings
  virtual std::optional<bool> GetOutputVoltageSetting();

 private:
  rclcpp::Node::SharedPtr node_handle_;

  std::vector<uint32_t> serials_;
  std::vector<FlyCapture2::Property> properties_;
  std::optional<std::pair<FlyCapture2::FrameRate, float> > frame_rate_;
  std::optional<FlyCapture2::VideoMode> video_mode_;
  std::optional<FlyCapture2::Format7ImageSettings> format7_setting_;
  std::optional<FlyCapture2::ColorProcessingAlgorithm> software_demosaicing_;
  std::optional<SelfTriggerSettings> self_trigger_setting_;
  std::optional<FlyCapture2::TriggerMode> trigger_mode_;
  std::optional<ImageType> image_type_;
  std::optional<bool> output_voltage_enable_;
  /// Mutex
  std::shared_mutex access_;
};

}  // end of namespace tmc_pgr_camera

#endif  // TMC_PGR_CAMERA_ROS_PARAMETER_POINT_GREY_CAMERA_SYSTEM_SETTING_HPP_
