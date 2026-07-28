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
/// @brief      Load configuration file for Point Grey camera system
#ifndef TMC_PGR_CAMERA_YAML_POINT_GREY_CAMERA_SYSTEM_SETTING_HPP_
#define TMC_PGR_CAMERA_YAML_POINT_GREY_CAMERA_SYSTEM_SETTING_HPP_
#include <any>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "tmc_pgr_camera/i_point_grey_camera_system_setting.hpp"

namespace tmc_pgr_camera {

/// @brief Class for loading Point Grey camera system settings from a YAML file
class YamlPointGreyCameraSystemSetting : public IPointGreyCameraSystemSetting {
 public:
  /// Constructor
  explicit YamlPointGreyCameraSystemSetting(const std::string& camera_setting_file_path);

  /// Retrieve the array of serial numbers for cameras used in the camera system
  virtual std::vector<uint32_t> GetSerialNumbers();

  /// Load the array of camera properties
  virtual std::vector<FlyCapture2::Property> GetProperties();

  /// Retrieve the frame rate
  virtual std::optional<std::pair<FlyCapture2::FrameRate, float> >
      GetFrameRate(const FlyCapture2::VideoMode video_mode);

  /// Retrieve the video mode
  virtual std::optional<FlyCapture2::VideoMode> GetVideoMode();

  /// Retrieve Format7 settings
  virtual std::optional<FlyCapture2::Format7ImageSettings> GetFormat7Setting();

  /// Retrieve software demosaicing settings
  virtual std::optional<FlyCapture2::ColorProcessingAlgorithm> GetSoftDemosaicing();

  /// Check if software trigger is enabled
  virtual bool IsSoftwareTriggerEnabled();

  /// Check if self-trigger is enabled
  virtual bool IsSelfTriggerEnabled();

  /// Retrieve self-trigger settings
  virtual std::optional<SelfTriggerSettings> GetSelfTriggerSettings();

  /// Retrieve trigger mode settings
  virtual std::optional<FlyCapture2::TriggerMode> GetTriggerMode();

  /// Create trigger mode settings
  virtual void UpdateTriggerMode(int /* trigger_mode_mode */,
                                 bool /* trigger_mode_on_off */,
                                 int /* trigger_mode_polarity */) {}

  /// Retrieve trigger delay settings
  virtual std::optional<FlyCapture2::TriggerDelay> GetTriggerDelay();

  /// Retrieve image type settings (monochrome, color)
  virtual std::optional<ImageType> GetImageType();

  /// Retrieve 3.3V output settings
  virtual std::optional<bool> GetOutputVoltageSetting();

 private:
  /// Node that loaded the configuration file
  YAML::Node setting_node_;

  /// Cache
  std::unordered_map<int32_t, std::any> cache_;
};

}  // end of namespace tmc_pgr_camera

#endif  // TMC_PGR_CAMERA_YAML_POINT_GREY_CAMERA_SYSTEM_SETTING_HPP_
