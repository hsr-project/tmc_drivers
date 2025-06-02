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
/// @brief Point Grey Camera
#ifndef TMC_PGR_CAMERA_POINT_GREY_CAMERA_HPP_
#define TMC_PGR_CAMERA_POINT_GREY_CAMERA_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include <flycapture/FlyCapture2.h>
#include <shared_mutex>

namespace tmc_pgr_camera {

/// @brief Point Grey Camera Class
class PointGreyCamera {
  PointGreyCamera(const PointGreyCamera&) = delete;
  PointGreyCamera& operator=(const PointGreyCamera&) = delete;

 public:
  /// Constructor
  explicit PointGreyCamera(const uint32_t serial_number);
  /// Destructor
  ~PointGreyCamera();

  /// Check if the camera is running
  bool IsOpened();

  /// Restart the camera
  void RestartCamera();

  /// Set properties
  void SetProperties(const std::vector<FlyCapture2::Property>& properties, const bool show_result);

  /// Configure video mode and frame rate
  void SetVideoModeAndFrameRate(const FlyCapture2::VideoMode video_mode,
                                const std::pair<FlyCapture2::FrameRate, float>& frame_rate);

  /// Configure RAW settings for camera output
  void SetFormat7Configuration(const FlyCapture2::Format7ImageSettings& format7_setting);

  /// Configure self-trigger
  void SetSelfTriggerSetting(const uint32_t out_io, const uint32_t pulse_figure);

  /// Configure trigger mode
  void SetTriggerMode(const FlyCapture2::TriggerMode& trigger_mode);

  /// Get trigger mode
  FlyCapture2::TriggerMode GetTriggerMode();

  /// Retrieve trigger mode information
  FlyCapture2::TriggerModeInfo GetTriggerModeInfo();

  /// Get software trigger state
  bool GetSoftwareTrigger();

  /// Configure trigger delay
  void SetTriggerDelay(const FlyCapture2::TriggerDelay& trigger_delay);

  /// Configure camera settings
  void SetConfiguration(const FlyCapture2::FC2Config& config);

  /// Configure software trigger
  bool SetSoftwareTrigger(const bool set_switch);

  /// Start capture
  void StartCapture();

  /// Stop capture
  void StopCapture();

  /// Start synchronized capture with multiple cameras
  static void StartSyncCapture(const std::vector<std::shared_ptr<PointGreyCamera> >& cameras);

  /// Acquire image
  FlyCapture2::Image RetrieveBuffer();

  /// Write value to camera register
  void WriteRegister(const uint32_t address, const uint32_t value, const bool broadcast);

  /// Read value from camera register
  void ReadRegister(const uint32_t address, uint32_t& value);

  /// Emit PWM waveform a specified number of times
  void SendPwmForSelfTrigger(const uint32_t out_io,
                             const uint32_t number_of_pulse,
                             const uint32_t polarity);

  /// Stop emitting PWM waveform
  void StopPwmForSelfTrigger(const uint32_t out_io);

  /// (Blackfly only) Output 3.3V from GPIO
  void OutputVoltage(const bool enable);

  /// Check if capture is happening
  bool is_capturing();

 private:
  /// Camera serial number
  uint32_t serial_number_;
  /// Mutex
  std::shared_mutex access_;
  /// Flag indicating if capture is happening
  bool is_capturing_;
  /// Camera object
  std::shared_ptr<FlyCapture2::Camera> camera_;
};

}  // end of namespace tmc_pgr_camera

#endif  // TMC_PGR_CAMERA_POINT_GREY_CAMERA_HPP_
