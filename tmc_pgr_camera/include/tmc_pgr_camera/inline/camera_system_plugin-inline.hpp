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
/// @brief      Camera system plugin template
#ifndef TMC_PGR_CAMERA_INLINE_CAMERA_SYSTEM_PLUGIN_INLINE_HPP_
#define TMC_PGR_CAMERA_INLINE_CAMERA_SYSTEM_PLUGIN_INLINE_HPP_

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <shared_mutex>

namespace tmc_pgr_camera {

class IPointGreyCameraSystemSetting;

/// @brief Constructor
template <class CameraSystem>
inline CameraSystemPlugin<CameraSystem>::CameraSystemPlugin()
    : camera_() {}

/// @brief Plugin initialization function
/// @param camera_setting_file_path Path to the camera settings file
template <class CameraSystem>
inline void CameraSystemPlugin<CameraSystem>::Initialize(
    std::shared_ptr<IPointGreyCameraSystemSetting>& camera_setting) {
  camera_.reset(new CameraSystem(camera_setting));
}

/// @brief Start the camera
/// @param camera_setting_file_path Path to the camera settings file
/// @exception std::runtime_error If called before initialization
template <class CameraSystem>
inline void CameraSystemPlugin<CameraSystem>::Open() {
  if (!camera_) {
    throw std::runtime_error("Call initialize function before using this function.");
  }
  camera_->Open();
}

/// @brief Stop the camera
/// @exception std::runtime_error If called before initialization
template <class CameraSystem>
inline void CameraSystemPlugin<CameraSystem>::Close() {
  if (!camera_) {
    throw std::runtime_error("Call initialize function before using this function.");
  }
  camera_->Close();
}

/// @brief Start capturing
/// @exception std::runtime_error If called before initialization
template <class CameraSystem>
inline void CameraSystemPlugin<CameraSystem>::StartCapture() {
  if (!camera_) {
    throw std::runtime_error("Call initialize function before using this function.");
  }
  camera_->StartCapture();
}

/// @brief Stop capturing
/// @exception std::runtime_error If called before initialization
template <class CameraSystem>
inline void CameraSystemPlugin<CameraSystem>::StopCapture() {
  if (!camera_) {
    throw std::runtime_error("Call initialize function before using this function.");
  }
  camera_->StopCapture();
}

/// @brief Retrieve captured images
/// @return Array of captured images (images for each camera)
/// @exception std::runtime_error If called before initialization
template <class CameraSystem>
inline std::optional<std::vector<ImagePtr> > CameraSystemPlugin<CameraSystem>::GrabImage() {
  if (!camera_) {
    throw std::runtime_error("Call initialize function before using this function.");
  }
  return camera_->GrabImage();
}

/// @brief Check if the camera is running
/// @return Returns true if running
/// @exception std::runtime_error If called before initialization
template <class CameraSystem>
inline bool CameraSystemPlugin<CameraSystem>::IsOpened() const {
  if (!camera_) {
    throw std::runtime_error("Call initialize function before using this function.");
  }
  return camera_->IsOpened();
}

/// @brief Check if capturing is in progress
/// @return Returns true if capturing is in progress
/// @exception std::runtime_error If called before initialization
template <class CameraSystem>
inline bool CameraSystemPlugin<CameraSystem>::IsCapturing() const {
  if (!camera_) {
    throw std::runtime_error("Call initialize function before using this function.");
  }
  return camera_->IsCapturing();
}

/// @brief Configure the camera
/// @exception std::runtime_error If called before initialization
template <class CameraSystem>
inline void CameraSystemPlugin<CameraSystem>::SetSettings(const YAML::Node& settings) {
  if (!camera_) {
    throw std::runtime_error("Call initialize function before using this function.");
  }
  return camera_->SetSettings(settings);
}

}  // end of namespace tmc_pgr_camera

#endif  // TMC_PGR_CAMERA_INLINE_CAMERA_SYSTEM_PLUGIN_INLINE_HPP_
