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
/// @brief      Point Grey camera
#include "tmc_pgr_camera/point_grey_camera.hpp"
#include <algorithm>
#include <array>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <boost/format.hpp>
#include <console_bridge/console.h>

namespace {

// Register address for initializing the camera
const uint32_t kInitializeAddress = 0x000;
// Value for initializing the camera
const uint32_t kInitializeValue = 0x80000000;

// Register address for starting the camera
const uint32_t kCameraPowerAddress = 0x610;
// Value for starting the camera
const uint32_t kCameraPowerValue = 0x80000000;

// Register address for setting the frame rate
const uint32_t kFrameRateAddress = 0x83C;

// Register address for setting the frame rate value
const uint32_t kAbsValFrameRateAddress = 0x968;

// Register address for controlling GPIO
const uint32_t kGpioCtrlPin0Address = 0x1110;
const uint32_t kGpioCtrlPin1Address = 0x1120;
const uint32_t kGpioCtrlPin2Address = 0x1130;
const uint32_t kGpioCtrlPin3Address = 0x1140;
// Value for controlling GPIO
const uint32_t kGpioCtrlPinValue = 0x80040000;

// Register address for registering PWM waveform
// 0, 1 for Chameleon, 2, 3 for Flea2
const uint32_t kGpioXtraPin0Address = 0x1114;
const uint32_t kGpioXtraPin1Address = 0x1124;
const uint32_t kGpioXtraPin2Address = 0x1134;
const uint32_t kGpioXtraPin3Address = 0x1144;

// Register address for setting software trigger
const uint32_t kSoftwareTriggerAddress = 0x62C;
// Value when setting software trigger
const uint32_t kSoftwareTriggerSetValue = 0x80000000;
// Value when resetting software trigger
const uint32_t kSoftwareTriggerResetValue = 0x00000000;
// Value when software trigger is in Busy state
const uint32_t kSoftwareTriggerStateBusy = 0x80000000;
// Value when software trigger is in Ready state
const uint32_t kSoftwareTriggerStateReady = 0x00000000;

// (Blackfly only) Register address for 3.3V output control
const uint32_t kOutputVoltageEnableAddress = 0x19D0;

// Register value when outputting 3.3V
const uint32_t kOutputVoltageEnableOn = 0x80000001;
// Register value when not outputting 3.3V
const uint32_t kOutputVoltageEnableOff = 0x80000000;
}  // anonymous namespace

namespace tmc_pgr_camera {

/// @brief Constructor
/// @param[in] serial_number Serial number of the camera to use
/// @exception std::runtime_error If initialization fails
PointGreyCamera::PointGreyCamera(const uint32_t serial_number)
    : serial_number_(serial_number), access_(), is_capturing_(false), camera_() {
  FlyCapture2::BusManager bus_manager;
  FlyCapture2::PGRGuid pgr_guid;
  FlyCapture2::Error error = bus_manager.GetCameraFromSerialNumber(serial_number, &pgr_guid);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error(
        (boost::format("Camera is not found. Serial Number: %1%\n%2%\n%3%") %
            serial_number %
            description %
            std::string("Kill this node, then type 'flycap' on console to see connected camera id.")).str());
  }
  camera_.reset(new FlyCapture2::Camera());
  error = camera_->Connect(&pgr_guid);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error(
        (boost::format("Failed to Connect to camera. Serial Number: %1%\n%2%") % serial_number % description).str());
  }
}

/// @brief Destructor
PointGreyCamera::~PointGreyCamera() {
  try {
    if (camera_) {
      camera_->Disconnect();
    }
  } catch (...) {
    // pass
  }
}

/// @brief Check if the camera is running
/// @return Returns true if running
/// @exception std::runtime_error If the camera pointer is null
bool PointGreyCamera::IsOpened() {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::shared_lock<std::shared_mutex> read(access_);
  return camera_->IsConnected();
}

/// @brief Restart the camera
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If restarting the camera fails
void PointGreyCamera::RestartCamera() {
  std::unique_lock<std::shared_mutex> write(access_);
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }

  // Reset the camera
  FlyCapture2::Error error = camera_->WriteRegister(kInitializeAddress, kInitializeValue);
  if (error != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to reset camera.\n" + description);
  }

  // Start the camera
  error = camera_->WriteRegister(kCameraPowerAddress, kCameraPowerValue);
  if (error != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to run camera.\n" + description);
  }

  // Wait until it starts
  uint32_t camera_power_status = 0x00000000;
  do {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    error = camera_->ReadRegister(kCameraPowerAddress, &camera_power_status);
    if (error != FlyCapture2::PGRERROR_OK) {
      const std::string description(error.GetDescription());
      throw std::runtime_error("Failed to restart camera.\n" + description);
    }
  } while ((camera_power_status & kCameraPowerValue) == 0);
}

/// @brief Check if the software trigger can be hit
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If state retrieval fails
/// @return bool Whether it can be triggered
bool PointGreyCamera::GetSoftwareTrigger() {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }

  std::unique_lock<std::shared_mutex> write(access_);

  uint32_t data = kSoftwareTriggerStateBusy;
  FlyCapture2::Error error = camera_->ReadRegister(kSoftwareTriggerAddress, &data);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to get self trigger setting.\n" + description);
  }

  return data == kSoftwareTriggerStateReady;
}

/// @brief Set properties
/// @param[in] properties Array of properties to set
/// @param[in] show_result Display on successful setting if true
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If camera properties could not be set
void PointGreyCamera::SetProperties(const std::vector<FlyCapture2::Property>& properties, const bool show_result) {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::unique_lock<std::shared_mutex> write(access_);

  std::unordered_map<uint32_t, std::string> types;
  types[static_cast<uint32_t>(FlyCapture2::BRIGHTNESS)] = "brightness";
  types[static_cast<uint32_t>(FlyCapture2::AUTO_EXPOSURE)] = "auto_exposure";
  types[static_cast<uint32_t>(FlyCapture2::SHARPNESS)] = "sharpness";
  types[static_cast<uint32_t>(FlyCapture2::WHITE_BALANCE)] = "white_balance";
  types[static_cast<uint32_t>(FlyCapture2::HUE)] = "hue";
  types[static_cast<uint32_t>(FlyCapture2::SATURATION)] = "saturation";
  types[static_cast<uint32_t>(FlyCapture2::GAMMA)] = "gamma";
  types[static_cast<uint32_t>(FlyCapture2::IRIS)] = "iris";
  types[static_cast<uint32_t>(FlyCapture2::FOCUS)] = "focus";
  types[static_cast<uint32_t>(FlyCapture2::ZOOM)] = "zoom";
  types[static_cast<uint32_t>(FlyCapture2::PAN)] = "pan";
  types[static_cast<uint32_t>(FlyCapture2::TILT)] = "tilt";
  types[static_cast<uint32_t>(FlyCapture2::SHUTTER)] = "shutter";
  types[static_cast<uint32_t>(FlyCapture2::GAIN)] = "gain";
  types[static_cast<uint32_t>(FlyCapture2::TRIGGER_MODE)] = "trigger_mode";
  types[static_cast<uint32_t>(FlyCapture2::TRIGGER_DELAY)] = "trigger_delay";
  types[static_cast<uint32_t>(FlyCapture2::FRAME_RATE)] = "frame_rate";
  types[static_cast<uint32_t>(FlyCapture2::TEMPERATURE)] = "temperature";

  for (const FlyCapture2::Property& property : properties) {
    FlyCapture2::Property setting_property = property;
    switch (setting_property.type) {
      case FlyCapture2::BRIGHTNESS:
      case FlyCapture2::AUTO_EXPOSURE:
      case FlyCapture2::SHARPNESS:
      case FlyCapture2::SHUTTER:
      case FlyCapture2::GAIN:
        if (setting_property.autoManualMode) {
          setting_property.absControl = false;
        } else {
          setting_property.absControl = true;
        }
        break;
    }

    const FlyCapture2::Error error = camera_->SetProperty(&setting_property);
    if (error.GetType() != FlyCapture2::PGRERROR_OK) {
      const std::string description(error.GetDescription());
      CONSOLE_BRIDGE_logWarn("Failed to set camera property '%s'.\n%s",
          types[static_cast<uint32_t>(setting_property.type)].c_str(), description.c_str());
    } else if (show_result) {
      CONSOLE_BRIDGE_logInform("Set camera property '%s'", types[static_cast<uint32_t>(setting_property.type)].c_str());
    }
  }
}

/// @brief Set video mode and frame rate
/// @param[in] video_mode Video mode
/// @param[in] frame_rate
///            Pair of constant value and actual value for frame rate in FlyCapture2 SDK
///            When the constant value for frame rate is FRAMERATE_FORMAT7
///            Use the actual value
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If setting fails
/// @note If setting fails with non-Format7 parameters
///       Only message output, no exception thrown
void PointGreyCamera::SetVideoModeAndFrameRate(const FlyCapture2::VideoMode video_mode,
                                               const std::pair<FlyCapture2::FrameRate, float>& frame_rate) {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::unique_lock<std::shared_mutex> write(access_);

  // In case of FORMAT7 (manufacturer's method)
  if (video_mode == FlyCapture2::VIDEOMODE_FORMAT7) {
    if (frame_rate.first != FlyCapture2::FRAMERATE_FORMAT7) {
      throw std::runtime_error(
          "Invalid frame rate. "
          "If you set video mode to 'VIDEOMODE_FORMAT7', "
          "you should set frame rate to 'FRAMERATE_FORMAT7'");
    }

    // Data of 83Ch
    uint32_t data = 0;
    // Retrieve data of 83Ch
    FlyCapture2::Error error = camera_->ReadRegister(kFrameRateAddress, &data);
    if (error.GetType() != FlyCapture2::PGRERROR_OK) {
      const std::string description(error.GetDescription());
      throw std::runtime_error("Failed to read camera frame rate settings.\n" + description);
    }
    // Set 1 for 0,1,6th bit of 83Ch, lower 3 bytes are retrieved register information, others are set to zero
    data &= 0xFFF;
    data |= 0xC2000000;

    // Set to 83Ch
    error = camera_->WriteRegister(kFrameRateAddress, data);
    if (error.GetType() != FlyCapture2::PGRERROR_OK) {
      const std::string description(error.GetDescription());
      throw std::runtime_error("Failed to set camera frame rate settings.\n" + description);
    }

    // Set frame rate to 968h
    union RegisterValue {
      uint32_t d_value;
      float f_value;
    };
    union RegisterValue value;
    value.f_value = frame_rate.second;
    error = camera_->WriteRegister(kAbsValFrameRateAddress, value.d_value);
    if (error.GetType() != FlyCapture2::PGRERROR_OK) {
      const std::string description(error.GetDescription());
      throw std::runtime_error("Failed to set camera frame rate.\n" + description);
    }

    // In case of non-FORMAT7
  } else {
    FlyCapture2::Error error = camera_->SetVideoModeAndFrameRate(video_mode, frame_rate.first);
    if (error.GetType() != FlyCapture2::PGRERROR_OK) {
      const std::string description(error.GetDescription());
      CONSOLE_BRIDGE_logWarn("Failed to set frame rate.\n%s", description.c_str());
    }
  }
}

/// @brief Set RAW settings for camera output
/// @param[in] format7_setting Value to set
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If setting fails
void PointGreyCamera::SetFormat7Configuration(const FlyCapture2::Format7ImageSettings& format7_setting) {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::unique_lock<std::shared_mutex> write(access_);

  FlyCapture2::Format7ImageSettings format7_image_settings;
  uint32_t packat_size = 0;
  float percentage = 0.0;
  FlyCapture2::Format7Info format7_info;
  // If previously set with non-Format7, retrieval of default value fails
  FlyCapture2::Error error = camera_->GetFormat7Configuration(&format7_image_settings, &packat_size, &percentage);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    // If not previously set with Format7
    // API fails, but it's not a critical error, so ignore
  } else {
    // If API succeeds, set the current mode retrieved from the camera
    // If failed, call camera information retrieval API with initial value FC2_MODE_0
    // (FC2_MODE_0 is the mode at camera startup)
    format7_info.mode = format7_image_settings.mode;
  }
  // Retrieve camera specifications
  bool supported = false;
  error = camera_->GetFormat7Info(&format7_info, &supported);
  // Abnormal if API fails or trying to retrieve unsupported mode setting information
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to get format7 information.\n" + description);
  }

  // Check if setting parameters are contained within camera specifications
  // If setting parameters are not contained within camera specifications, make adjustments
  uint32_t width = std::min(format7_setting.width, format7_info.maxWidth);
  width -= width % format7_info.imageHStepSize;
  uint32_t height = std::min(format7_setting.height, format7_info.maxHeight);
  height -= height % format7_info.imageVStepSize;

  // Determine the maximum possible value for width and height offset
  const uint32_t max_offset_x = format7_info.maxWidth - width;
  uint32_t offset_x = std::min(format7_setting.offsetX, max_offset_x);
  offset_x -= offset_x % format7_info.offsetHStepSize;
  const uint32_t max_offset_y = format7_info.maxHeight - height;
  uint32_t offset_y = std::min(format7_setting.offsetY, max_offset_y);
  offset_y -= offset_y % format7_info.offsetVStepSize;

  format7_image_settings.width = width;
  format7_image_settings.height = height;
  format7_image_settings.offsetX = offset_x;
  format7_image_settings.offsetY = offset_y;
  format7_image_settings.mode = format7_setting.mode;
  format7_image_settings.pixelFormat = format7_setting.pixelFormat;

  // Determine if setting information is valid for the camera
  // Mainly check mode and pixel_format
  FlyCapture2::Format7PacketInfo format7_packet_info;
  error = camera_->ValidateFormat7Settings(&format7_image_settings, &supported, &format7_packet_info);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to validate format7 settings.\n" + description);
  }
  if (!supported) {
    CONSOLE_BRIDGE_logWarn("Not supported parameter. Therefore, set mode to 0 and pixel format to raw8");
    format7_image_settings.mode = FlyCapture2::MODE_0;
    format7_image_settings.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW8;
  }

  // Determine Byte Per Pixel
  const uint32_t bits_per_pixel = FlyCapture2::Image::DetermineBitsPerPixel(format7_image_settings.pixelFormat);

  // Retrieve frame rate
  union RegisterValue {
    uint32_t d_value;
    float f_value;
  };
  union RegisterValue fps;
  error = camera_->ReadRegister(kAbsValFrameRateAddress, &fps.d_value);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to read frame rate.\n" + description);
  }

  // Determine Bytes Per Packet
  const float cycles_per_second = 8000.0;
  const float frames_per_second = fps.f_value;
  const float packets_per_frame = cycles_per_second / frames_per_second;
  const float bytes_per_frame = (format7_image_settings.width * format7_image_settings.height) * (bits_per_pixel / 8.0);
  const float bytes_per_packet = bytes_per_frame / packets_per_frame;

  if (format7_packet_info.maxBytesPerPacket == 0) {
    throw std::runtime_error("Division by zero. Max bytes per packet is zero.");
  }
  // Determine PercentSpeed
  const float percent_speed = 100.0 * bytes_per_packet / format7_packet_info.maxBytesPerPacket;

  // RAW information setting
  error = camera_->SetFormat7Configuration(&format7_image_settings, percent_speed);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to set format7 configuration.\n" + description);
  }
}

/// @brief Set self-trigger
/// @param[in] out_io Output register address
///                   Specify 3 for Flea2
///                   Specify 1 for Chameleon
/// @param[in] pulse_figure Self-trigger pulse emission command
/// @exception std::invalid_argument If out_io value is incorrect
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If setting fails
void PointGreyCamera::SetSelfTriggerSetting(const uint32_t out_io, const uint32_t pulse_figure) {
  if (out_io != 3 && out_io != 1) {
    throw std::invalid_argument("Invalid 'out_io' value.");
  }
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::unique_lock<std::shared_mutex> write(access_);
  const std::array<uint32_t, 4> addresses = { kGpioXtraPin0Address, kGpioXtraPin1Address, kGpioXtraPin2Address,
                                              kGpioXtraPin3Address };
  FlyCapture2::Error error = camera_->WriteRegister(addresses.at(out_io), pulse_figure);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to set self trigger setting.\n" + description);
  }
}

/// @brief Set trigger mode
/// @param[in] trigger_mode Camera mode
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If setting fails
void PointGreyCamera::SetTriggerMode(const FlyCapture2::TriggerMode& trigger_mode) {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::unique_lock<std::shared_mutex> write(access_);
  const FlyCapture2::Error error = camera_->SetTriggerMode(&trigger_mode);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to set trigger mode.\n" + description);
  }
}

/// @brief Retrieve trigger mode
/// @return Retrieved trigger mode
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If retrieval fails
FlyCapture2::TriggerMode PointGreyCamera::GetTriggerMode() {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::shared_lock<std::shared_mutex> read(access_);
  FlyCapture2::TriggerMode trigger_mode;
  const FlyCapture2::Error error = camera_->GetTriggerMode(&trigger_mode);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to get trigger mode.\n" + description);
  }
  return trigger_mode;
}

/// @brief Retrieve trigger mode information
/// @return Retrieved trigger mode information
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If retrieval fails
FlyCapture2::TriggerModeInfo PointGreyCamera::GetTriggerModeInfo() {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::shared_lock<std::shared_mutex> read(access_);
  FlyCapture2::TriggerModeInfo trigger_mode_info;
  const FlyCapture2::Error error = camera_->GetTriggerModeInfo(&trigger_mode_info);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to get trigger mode info.\n" + description);
  }
  return trigger_mode_info;
}

/// @brief Set trigger delay
/// @param[in] trigger_delay Trigger delay setting
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If setting fails
void PointGreyCamera::SetTriggerDelay(const FlyCapture2::TriggerDelay& trigger_delay) {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::unique_lock<std::shared_mutex> write(access_);
  FlyCapture2::TriggerDelay tmp_trigger_delay;
  FlyCapture2::Error error = camera_->GetTriggerDelay(&tmp_trigger_delay);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to get trigger delay.\n" + description);
  }
  tmp_trigger_delay.absValue = trigger_delay.absValue;
  tmp_trigger_delay.absControl = trigger_delay.absControl;
  error = camera_->SetTriggerDelay(&tmp_trigger_delay);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to set trigger delay.\n" + description);
  }
}

/// @brief Set camera settings
/// @param[in] config Camera settings
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If setting fails
void PointGreyCamera::SetConfiguration(const FlyCapture2::FC2Config& config) {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::unique_lock<std::shared_mutex> write(access_);
  const FlyCapture2::Error error = camera_->SetConfiguration(&config);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to set configuration.\n" + description);
  }
}

/// @brief Set software trigger
/// @param[in] set_switch Set software trigger if true, reset if false
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If setting fails
/// @return bool Whether setting was successful
bool PointGreyCamera::SetSoftwareTrigger(const bool set_switch) {
  if (!camera_) {
    std::cerr << "Camera does not exist." << std::endl;
    throw std::runtime_error("Camera does not exist.");
  }
  if (!GetSoftwareTrigger() && set_switch) {
    return false;
  }
  std::unique_lock<std::shared_mutex> write(access_);
  const FlyCapture2::Error error = camera_->WriteRegister(
      kSoftwareTriggerAddress, set_switch ? kSoftwareTriggerSetValue : kSoftwareTriggerResetValue, true);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to set configuration.\n" + description);
  }
  return true;
}

/// @brief Start capture
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If starting capture fails
void PointGreyCamera::StartCapture() {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::unique_lock<std::shared_mutex> write(access_);
  const FlyCapture2::Error error = camera_->StartCapture();
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to start capture.\n" + description);
  }
  is_capturing_ = true;
}

/// @brief Stop capture
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If stopping capture fails
void PointGreyCamera::StopCapture() {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::unique_lock<std::shared_mutex> write(access_);
  const FlyCapture2::Error error = camera_->StopCapture();
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to stop capture.\n" + description);
  }
  is_capturing_ = false;
}

/// @brief Start synchronized capture with multiple cameras
/// @param[in] cameras Array of cameras to synchronize
/// @exception std::invalid_argument If the camera array is empty
/// @exception std::runtime_error If starting capture fails
void PointGreyCamera::StartSyncCapture(const std::vector<std::shared_ptr<PointGreyCamera> >& cameras) {
  if (cameras.empty()) {
    throw std::invalid_argument("There are no cameras.");
  }

  std::vector<std::shared_ptr<std::unique_lock<std::shared_mutex> > > writes;
  std::vector<const FlyCapture2::Camera*> camera_ptrs;
  typedef std::shared_ptr<PointGreyCamera> CameraPtr;
  for (const CameraPtr& camera : cameras) {
    writes.push_back(std::shared_ptr<std::unique_lock<std::shared_mutex> >(
        new std::unique_lock<std::shared_mutex>(camera->access_)));
    camera_ptrs.push_back(camera->camera_.get());
  }

  const FlyCapture2::Error error = FlyCapture2::Camera::StartSyncCapture(camera_ptrs.size(), &camera_ptrs.at(0));
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to start sync capture.\n" + description);
  }

  for (const CameraPtr& camera : cameras) { camera->is_capturing_ = true; }
}

/// @brief Retrieve image
/// @return Retrieved image
/// @exception If capture fails
/// @exception std::runtime_error If the camera pointer is null
/// @note Call after starting capture
FlyCapture2::Image PointGreyCamera::RetrieveBuffer() {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }

  if (!is_capturing_) {
    throw std::runtime_error("Capture did not start.");
  }

  FlyCapture2::Image image;
  const FlyCapture2::Error error = camera_->RetrieveBuffer(&image);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to retrieve buffer.\n" + description);
  }

  return image;
}

/// @brief Write value to camera register
/// @param[in] address Address to write to
/// @param[in] value Value to write
/// @param[in] broadcast Flag for using broadcast
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If writing fails
void PointGreyCamera::WriteRegister(const uint32_t address, const uint32_t value, const bool broadcast) {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::unique_lock<std::shared_mutex> write(access_);
  const FlyCapture2::Error error = camera_->WriteRegister(address, value, broadcast);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to write register.\n" + description);
  }
}

/// @brief Read value from camera register
/// @param[in] address Address to read from
/// @param[out] value Read value
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If reading fails
void PointGreyCamera::ReadRegister(const uint32_t address, uint32_t& value) {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::unique_lock<std::shared_mutex> write(access_);
  const FlyCapture2::Error error = camera_->ReadRegister(address, &value);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to write register.\n" + description);
  }
}

/// @brief Emit PWM waveform a specified number of times
/// @param[in] out_io Location to output waveform
/// @param[in] number_of_pulse Number of times to output
/// @param[in] polarity Polarity of waveform
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If emission fails
void PointGreyCamera::SendPwmForSelfTrigger(const uint32_t out_io,
                                            const uint32_t number_of_pulse,
                                            const uint32_t polarity) {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::unique_lock<std::shared_mutex> write(access_);
  const std::array<uint32_t, 4> addresses = { kGpioCtrlPin0Address, kGpioCtrlPin1Address, kGpioCtrlPin2Address,
                                              kGpioCtrlPin3Address };
  const FlyCapture2::Error error =
      camera_->WriteRegister(addresses.at(out_io), kGpioCtrlPinValue | (number_of_pulse << 8) | polarity);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to send PWM.\n" + description);
  }
}

/// @brief Stop emitting PWM waveform
/// @param[in] out_io Location to output waveform
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If stopping fails
void PointGreyCamera::StopPwmForSelfTrigger(const uint32_t out_io) {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::unique_lock<std::shared_mutex> write(access_);
  const std::array<uint32_t, 4> addresses = { kGpioCtrlPin0Address, kGpioCtrlPin1Address, kGpioCtrlPin2Address,
                                              kGpioCtrlPin3Address };
  const FlyCapture2::Error error = camera_->WriteRegister(addresses.at(out_io), kGpioCtrlPinValue);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to stop PWM.\n" + description);
  }
}

/// @brief (Blackfly only) Output 3.3V from GPIO
/// @param[in] enable Output if True, stop if False
/// @exception std::runtime_error If the camera pointer is null
/// @exception std::runtime_error If setting fails
void PointGreyCamera::OutputVoltage(const bool enable) {
  if (!camera_) {
    throw std::runtime_error("Camera does not exist.");
  }
  std::unique_lock<std::shared_mutex> write(access_);
  uint32_t read_value;
  camera_->ReadRegister(kOutputVoltageEnableAddress, &read_value);
  if (read_value == 0) {
    CONSOLE_BRIDGE_logWarn("Output voltage function is not presented.");
    return;
  }
  const uint32_t value = enable ? kOutputVoltageEnableOn : kOutputVoltageEnableOff;
  const FlyCapture2::Error error = camera_->WriteRegister(kOutputVoltageEnableAddress, value);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    const std::string description(error.GetDescription());
    throw std::runtime_error("Failed to set output voltage setting.\n" + description);
  }
}

/// @brief Check if capture is being done
/// @return Returns true if capturing
bool PointGreyCamera::is_capturing() {
  std::shared_lock<std::shared_mutex> read(access_);
  return is_capturing_;
}

}  // end of namespace tmc_pgr_camera
