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
/// @brief Camera stub for testing

#include <unordered_map>
#include <utility>
#include <vector>
#include "tmc_pgr_camera/point_grey_camera.hpp"

namespace {

/// @brief Structure to save parameters used in the camera stub
struct CameraParameter {
 public:
  /// Map to save whether the camera is running
  static std::unordered_map<uint32_t, bool> s_is_opened_map;
};

std::unordered_map<uint32_t, bool> CameraParameter::s_is_opened_map;
}  // anonymous namespace

namespace tmc_pgr_camera {

/// @brief Constructor
/// @param[in] serial_number Serial number of the camera to use
/// @exception std::runtime_error If initialization fails
PointGreyCamera::PointGreyCamera(const uint32_t serial_number)
    : serial_number_(serial_number), access_(), is_capturing_(false), camera_() {
  CameraParameter::s_is_opened_map[serial_number_] = true;
}

/// @brief Destructor
PointGreyCamera::~PointGreyCamera() {
  try {
    CameraParameter::s_is_opened_map[serial_number_] = false;
  } catch (...) {
    // pass
  }
}

/// @brief Check if the camera is running
/// @return Returns true if running
/// @exception std::runtime_error If the camera pointer is null
bool PointGreyCamera::IsOpened() { return CameraParameter::s_is_opened_map[serial_number_]; }

/// @brief Restart the camera
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If restarting the camera fails
void PointGreyCamera::RestartCamera() {}

/// @brief Set properties
/// @param[in] properties Array of properties to set
/// @param[in] show_result Display on successful setting if true
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If camera properties could not be set
void PointGreyCamera::SetProperties(const std::vector<FlyCapture2::Property>& properties, const bool show_result) {}

/// @brief Set video mode and frame rate
/// @param[in] video_mode Video mode
/// @param[in] frame_rate Pair of constant value and actual value for frame rate in FlyCapture2 SDK
/// Use actual value when the constant value for frame rate is FRAMERATE_FORMAT7
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If setting fails
/// @note If setting fails with non-Format7 parameters, only message output is done without throwing an exception
/// @brief Set RAW settings for camera output
/// @param[in] format7_setting Value to set
/// @exception std::runtime_error If the camera pointer is null
void PointGreyCamera::SetVideoModeAndFrameRate(const FlyCapture2::VideoMode video_mode,
                                               const std::pair<FlyCapture2::FrameRate, float>& frame_rate) {}

/// @exception std::runtime_error If setting fails
/// @brief Set self-trigger
/// @param[in] out_io Output register address
/// Specify 3 for Flea2
void PointGreyCamera::SetFormat7Configuration(const FlyCapture2::Format7ImageSettings& format7_setting) {}

/// Specify 1 for Chameleon
/// @param[in] pulse_figure Self-trigger pulse transmission command
/// @exception std::invalid_argument If the value of out_io is incorrect
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If setting fails
/// @brief Set trigger mode
/// @param[in] trigger_mode Camera mode
/// @exception std::runtime_error If the camera pointer is null
void PointGreyCamera::SetSelfTriggerSetting(const uint32_t out_io, const uint32_t pulse_figure) {}

/// @exception std::runtime_error If setting fails
/// @brief Get trigger mode
/// @return Acquired trigger mode
/// @exception std::runtime_error If the camera pointer is null
void PointGreyCamera::SetTriggerMode(const FlyCapture2::TriggerMode& trigger_mode) {}

/// @exception std::runtime_error If acquisition fails
/// @brief Set trigger delay
/// @param[in] trigger_delay Trigger delay setting
/// @exception std::runtime_error If the camera pointer is null
FlyCapture2::TriggerMode PointGreyCamera::GetTriggerMode() {
  FlyCapture2::TriggerMode trigger_mode;
  trigger_mode.onOff = true;
  return trigger_mode;
}

/// @exception std::runtime_error If setting fails
/// @brief Set camera settings
/// @param[in] config Camera settings
/// @exception std::runtime_error If the camera pointer is null
void PointGreyCamera::SetTriggerDelay(const FlyCapture2::TriggerDelay& trigger_delay) {}

/// @exception std::runtime_error If setting fails
/// @brief Set software trigger
/// @param[in] set_switch Set software trigger if true, reset if false
/// @return Whether the software trigger process was successful
void PointGreyCamera::SetConfiguration(const FlyCapture2::FC2Config& config) {}

/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If setting fails
/// @brief Start capture
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If starting capture fails
bool PointGreyCamera::SetSoftwareTrigger(const bool set_switch) {}

/// @brief Stop capture
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If stopping capture fails
void PointGreyCamera::StartCapture() { is_capturing_ = true; }

/// @brief Start synchronized capture with multiple cameras
/// @param[in] cameras Array of cameras to synchronize
/// @exception std::invalid_argument If the camera array is empty
void PointGreyCamera::StopCapture() { is_capturing_ = false; }

/// @exception std::runtime_error If starting capture fails
/// @brief Acquire image
/// @return Acquired image
/// @exception If capture fails
void PointGreyCamera::StartSyncCapture(const std::vector<std::shared_ptr<PointGreyCamera> >& cameras) {
  for (const std::shared_ptr<PointGreyCamera>& camera : cameras) { camera->is_capturing_ = true; }
}

/// @exception std::runtime_error If the camera pointer is null
/// @note Call after starting capture
/// @brief Write value to camera register
/// @param[in] address Address to write to
/// @param[in] value Value to write
FlyCapture2::Image PointGreyCamera::RetrieveBuffer() {
  return FlyCapture2::Image(960, 1280, FlyCapture2::PIXEL_FORMAT_RAW8);
}

/// @param[in] broadcast Flag to use broadcast
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If writing fails
/// @brief Transmit PWM waveform a specified number of times
/// @param[in] out_io Location to output waveform
/// @param[in] number_of_pulse Number of times to output
void PointGreyCamera::WriteRegister(const uint32_t address, const uint32_t value, const bool broadcast) {}

/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If transmission fails
/// @brief Stop PWM waveform transmission
/// @param[in] out_io Location to output waveform
/// @exception std::runtime_error If the camera pointer is null
void PointGreyCamera::SendPwmForSelfTrigger(const uint32_t out_io,
                                            const uint32_t number_of_pulse,
                                            const uint32_t polarity) {}

/// @exception std::runtime_error If stopping fails
/// @brief (Blackfly only) Output 3.3V from GPIO
/// @param[in] enable Output if True, stop if False
/// @exception std::runtime_error If the camera pointer is null
void PointGreyCamera::StopPwmForSelfTrigger(const uint32_t out_io) {}

/// @exception std::runtime_error If setting fails
/// @brief Check if capture is in progress
/// @return Returns true if capturing
/// @exception std::runtime_error If the configuration fails
void PointGreyCamera::OutputVoltage(const bool enable) {}

/// @brief Check whether it is being captured
/// @return Returns true if captured
bool PointGreyCamera::is_capturing() { return is_capturing_; }

}  // end of namespace tmc_pgr_camera
