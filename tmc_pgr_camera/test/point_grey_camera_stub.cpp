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
/// @brief     Camera stub for testing

#include <unordered_map>
#include <utility>
#include <vector>
#include "tmc_pgr_camera/point_grey_camera.hpp"

namespace {

/// @brief Structure that saves parameters used in camera stub
struct CameraParameter {
 public:
  /// Map that saves whether the camera is running
  static std::unordered_map<uint32_t, bool> s_is_opened_map;
};

std::unordered_map<uint32_t, bool> CameraParameter::s_is_opened_map;
}  // anonymous namespace

namespace tmc_pgr_camera {

/// @brief Constructor
/// @param[in] serial_number Serial number of the camera to use
/// @exception std::runtime_error Failed to initialize
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
/// @return True if running
/// @exception std::runtime_error The camera pointer is empty
bool PointGreyCamera::IsOpened() { return CameraParameter::s_is_opened_map[serial_number_]; }

/// @brief Restart the camera
/// @exception std::runtime_error The camera pointer is empty
/// @exception std::runtime_error Failed to restart the camera
void PointGreyCamera::RestartCamera() {}

/// @brief Set properties
/// @param[in] properties Array of properties to set
/// @param[in] show_result Display on successful setting if true
/// @exception std::runtime_error The camera pointer is empty
/// @exception std::runtime_error Failed to set camera properties
void PointGreyCamera::SetProperties(const std::vector<FlyCapture2::Property>& properties, const bool show_result) {}

/// @brief Set video mode and frame rate
/// @param[in] video_mode Video mode
/// @param[in] frame_rate Pair of constant value and actual value in
///                       FlyCapture2 SDK for frame rate
///                   Use actual value when frame rate constant
///                   value is FRAMERATE_FORMAT7
/// @exception std::runtime_error The camera pointer is empty
/// @exception std::runtime_error Failed to set
/// @note If setting with non-Format7 parameters fails,
///       only messages are output without throwing exceptions
void PointGreyCamera::SetVideoModeAndFrameRate(const FlyCapture2::VideoMode video_mode,
                                               const std::pair<FlyCapture2::FrameRate, float>& frame_rate) {}

/// @brief Set RAW settings for camera output
/// @param[in] format7_setting Value to be set
/// @exception std::runtime_error The camera pointer is empty
/// @exception std::runtime_error Failed to set
void PointGreyCamera::SetFormat7Configuration(const FlyCapture2::Format7ImageSettings& format7_setting) {}

/// @brief Set self-trigger
/// @param[in] out_io Output register address
///        Specify 3 for Flea2
///        Specify 1 for Chameleon
/// @param[in] pulse_figure Self-trigger pulse command
/// @exception std::invalid_argument Invalid out_io value
/// @exception std::runtime_error The camera pointer is empty
/// @exception std::runtime_error Failed to set
void PointGreyCamera::SetSelfTriggerSetting(const uint32_t out_io, const uint32_t pulse_figure) {}

/// @brief Set trigger mode
/// @param[in] trigger_mode Camera mode
/// @exception std::runtime_error The camera pointer is empty
/// @exception std::runtime_error Failed to set
void PointGreyCamera::SetTriggerMode(const FlyCapture2::TriggerMode& trigger_mode) {}

/// @brief Obtain trigger mode
/// @return Obtained trigger mode
/// @exception std::runtime_error The camera pointer is empty
/// @exception std::runtime_error Failed to obtain
FlyCapture2::TriggerMode PointGreyCamera::GetTriggerMode() {
  FlyCapture2::TriggerMode trigger_mode;
  trigger_mode.onOff = true;
  return trigger_mode;
}

/// @brief Set trigger delay
/// @param[in] trigger_delay Trigger delay setting
/// @exception std::runtime_error The camera pointer is empty
/// @exception std::runtime_error Failed to set
void PointGreyCamera::SetTriggerDelay(const FlyCapture2::TriggerDelay& trigger_delay) {}

/// @brief Set camera settings
/// @param[in] config Camera settings
/// @exception std::runtime_error The camera pointer is empty
/// @exception std::runtime_error Failed to set
void PointGreyCamera::SetConfiguration(const FlyCapture2::FC2Config& config) {}

/// @brief Set software trigger
/// @param[in] set_switch Set software trigger when true, reset when false
/// @return Whether software trigger processing was successful
/// @exception std::runtime_error The camera pointer is empty
/// @exception std::runtime_error Failed to set
bool PointGreyCamera::SetSoftwareTrigger(const bool set_switch) {}

/// @brief Start capture
/// @exception std::runtime_error The camera pointer is empty
/// @exception std::runtime_error Failed to start capture
void PointGreyCamera::StartCapture() { is_capturing_ = true; }

/// @brief Stop capture
/// @exception std::runtime_error The camera pointer is empty
/// @exception std::runtime_error Failed to stop capture
void PointGreyCamera::StopCapture() { is_capturing_ = false; }

/// @brief Start synchronized capture with multiple cameras
/// @param[in] cameras Array of cameras to synchronize
/// @exception std::invalid_argument Camera array is empty
/// @exception std::runtime_error Failed to start capture
void PointGreyCamera::StartSyncCapture(const std::vector<std::shared_ptr<PointGreyCamera> >& cameras) {
  for (const std::shared_ptr<PointGreyCamera>& camera : cameras) { camera->is_capturing_ = true; }
}

/// @brief Obtain image
/// @return Obtained image
/// @exception Failed to capture
/// @exception std::runtime_error The camera pointer is empty
/// @note Call after starting capture
FlyCapture2::Image PointGreyCamera::RetrieveBuffer() {
  return FlyCapture2::Image(960, 1280, FlyCapture2::PIXEL_FORMAT_RAW8);
}

/// @brief Write values to the camera's registers
/// @param[in] address Address to write to
/// @param[in] value Value to write
/// @param[in] broadcast Flag to use broadcast
/// @exception std::runtime_error The camera pointer is empty
/// @exception std::runtime_error Failed to write
void PointGreyCamera::WriteRegister(const uint32_t address, const uint32_t value, const bool broadcast) {}

/// @brief Issue PWM waveform for specified number of times
/// @param[in] out_io Location to output waveform
/// @param[in] number_of_pulse Number of times to output
/// @exception std::runtime_error The camera pointer is empty
/// @exception std::runtime_error Failed to issue
void PointGreyCamera::SendPwmForSelfTrigger(const uint32_t out_io,
                                            const uint32_t number_of_pulse,
                                            const uint32_t polarity) {}

/// @brief Stop issuing PWM waveform
/// @param[in] out_io Location to output waveform
/// @exception std::runtime_error The camera pointer is empty
/// @exception std::runtime_error Failed to stop
void PointGreyCamera::StopPwmForSelfTrigger(const uint32_t out_io) {}

/// @brief (Blackfly only) Output 3.3V from GPIO
/// @param[in] enable True to output, False to stop
/// @exception std::runtime_error The camera pointer is empty
/// @exception std::runtime_error Failed to set
void PointGreyCamera::OutputVoltage(const bool enable) {}

/// @brief Check if capture is being done
/// @return True if captured
bool PointGreyCamera::is_capturing() { return is_capturing_; }

}  // end of namespace tmc_pgr_camera
