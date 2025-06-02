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
/// @brief Interface for the camera system plugin
#ifndef TMC_PGR_CAMERA_I_CAMERA_SYSTEM_PLUGIN_BASE_HPP_
#define TMC_PGR_CAMERA_I_CAMERA_SYSTEM_PLUGIN_BASE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <yaml-cpp/yaml.h>
#include "tmc_pgr_camera/image.hpp"

namespace tmc_pgr_camera {

class IPointGreyCameraSystemSetting;

/// @brief Interface class for the camera system plugin
class ICameraSystemPluginBase {
  ICameraSystemPluginBase(const ICameraSystemPluginBase&) = delete;
  ICameraSystemPluginBase& operator=(const ICameraSystemPluginBase&) = delete;

 public:
  /// Destructor
  virtual ~ICameraSystemPluginBase() {}

  /// Plugin initialization function
  virtual void Initialize(std::shared_ptr<IPointGreyCameraSystemSetting>& camera_setting) = 0;

  /// Start the camera system
  virtual void Open() = 0;
  /// Shutdown the camera system
  virtual void Close() = 0;

  /// Start capturing
  virtual void StartCapture() = 0;
  /// Stop capturing
  virtual void StopCapture() = 0;

  /// Retrieve capture image
  virtual std::optional<std::vector<ImagePtr> > GrabImage() = 0;

  /// Check if the camera system is running
  virtual bool IsOpened() const = 0;

  /// Check if capturing is ongoing
  virtual bool IsCapturing() const = 0;

  /// Configure the camera
  virtual void SetSettings(const YAML::Node& settings) = 0;

 protected:
  /// Constructor
  ICameraSystemPluginBase() {}
};

}  // end of namespace tmc_pgr_camera

#endif  // TMC_PGR_CAMERA_I_CAMERA_SYSTEM_PLUGIN_BASE_HPP_
