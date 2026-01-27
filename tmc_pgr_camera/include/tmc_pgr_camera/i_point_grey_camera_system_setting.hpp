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
/// @brief Interface for loading configuration files of the Point Grey camera system
#ifndef TMC_PGR_CAMERA_I_POINT_GREY_CAMERA_SYSTEM_SETTING_HPP_
#define TMC_PGR_CAMERA_I_POINT_GREY_CAMERA_SYSTEM_SETTING_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <flycapture/FlyCapture2.h>
#include <yaml-cpp/yaml.h>

namespace tmc_pgr_camera {

/// @brief Configuration structure for self-triggering
typedef struct SelfTriggerSettings {
  /// Input register address
  uint32_t in_io;
  /// Output register address
  uint32_t out_io;
  /// Self-trigger pulse transmission command
  uint32_t pulse_figure;
  /// Number of pulses
  uint32_t number_of_pulse;
  /// Polarity
  uint32_t polarity;
} SelfTriggerSettings;

/// @brief Enumeration representing image types
enum ImageType {
  /// Monochrome image
  kMonoImage,
  /// RGB image
  kRgbImage
};

/// @brief Class for loading settings of the Point Grey camera system
class IPointGreyCameraSystemSetting {
 public:
  /// Retrieve an array of serial numbers of cameras used in the camera system
  virtual std::vector<uint32_t> GetSerialNumbers() = 0;

  /// Load an array of camera properties
  virtual std::vector<FlyCapture2::Property> GetProperties() = 0;

  /// Retrieve the frame rate
  virtual std::optional<std::pair<FlyCapture2::FrameRate, float> >
      GetFrameRate(const FlyCapture2::VideoMode video_mode) = 0;

  /// Retrieve the video mode
  virtual std::optional<FlyCapture2::VideoMode> GetVideoMode() = 0;

  /// Retrieve the Format7 settings
  virtual std::optional<FlyCapture2::Format7ImageSettings> GetFormat7Setting() = 0;

  /// Retrieve the software demosaicing settings
  virtual std::optional<FlyCapture2::ColorProcessingAlgorithm> GetSoftDemosaicing() = 0;

  /// Check if the software trigger is enabled
  virtual bool IsSoftwareTriggerEnabled() = 0;

  /// Check if the self-trigger is enabled
  virtual bool IsSelfTriggerEnabled() = 0;

  /// Retrieve the self-trigger settings
  virtual std::optional<SelfTriggerSettings> GetSelfTriggerSettings() = 0;

  /// Retrieve the trigger mode settings
  virtual std::optional<FlyCapture2::TriggerMode> GetTriggerMode() = 0;

  /// Retrieve the trigger delay settings
  virtual std::optional<FlyCapture2::TriggerDelay> GetTriggerDelay() = 0;

  /// Create trigger mode settings
  virtual void UpdateTriggerMode(int trigger_mode_mode, bool trigger_mode_on_off, int trigger_mode_polarity) = 0;

  /// Retrieve the image type settings (monochrome, color)
  virtual std::optional<ImageType> GetImageType() = 0;

  /// Retrieve the 3.3V output settings
  virtual std::optional<bool> GetOutputVoltageSetting() = 0;
};

}  // end of namespace tmc_pgr_camera

#endif  // TMC_PGR_CAMERA_I_POINT_GREY_CAMERA_SYSTEM_SETTING_HPP_
