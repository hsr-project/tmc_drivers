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
/// @brief Point Grey camera system
#ifndef TMC_PGR_CAMERA_POINT_GREY_CAMERA_SYSTEM_HPP_
#define TMC_PGR_CAMERA_POINT_GREY_CAMERA_SYSTEM_HPP_

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <boost/circular_buffer.hpp>
#include <shared_mutex>
#include <yaml-cpp/yaml.h>

#include "tmc_pgr_camera/image.hpp"

namespace tmc_pgr_camera {

class PointGreyCamera;
class IPointGreyCameraSystemSetting;

/// @brief Point Grey camera system class
class PointGreyCameraSystem {
  PointGreyCameraSystem(const PointGreyCameraSystem&) = delete;
  PointGreyCameraSystem& operator=(const PointGreyCameraSystem&) = delete;

 public:
  /// Constructor
  explicit PointGreyCameraSystem(std::shared_ptr<IPointGreyCameraSystemSetting>& camera_system_setting);

  /// Destructor
  ~PointGreyCameraSystem();

  /// Start the camera system
  void Open();

  /// Shut down the camera system
  void Close();

  /// Start capture
  void StartCapture();
  /// Stop capture
  void StopCapture();

  /// Obtain captured image
  std::optional<std::vector<ImagePtr> > GrabImage();

  /// Check if the camera system is running
  bool IsOpened() const;

  /// Check if capturing is ongoing
  bool IsCapturing() const;

  /// Configure the camera settings
  void SetSettings(const YAML::Node& settings);

  /// Capture thread
  void CaptureThread();

 private:
  /// Array of Point Grey cameras
  std::vector<std::shared_ptr<PointGreyCamera> > cameras_;
  /// Capture thread
  std::thread capture_thread_;
  /// Buffer for captured images
  boost::circular_buffer<std::vector<ImagePtr> > captured_images_;
  /// Settings loading object
  std::shared_ptr<IPointGreyCameraSystemSetting> camera_system_settings_;
  /// Flag to terminate capture thread
  bool can_close_capture_thread_;
  /// Mutex
  std::shared_mutex access_;
};

}  // end of namespace tmc_pgr_camera

#endif  // TMC_PGR_CAMERA_POINT_GREY_CAMERA_SYSTEM_HPP_
