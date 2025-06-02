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
#include "tmc_pgr_camera/point_grey_camera_system.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <boost/format.hpp>
#include <console_bridge/console.h>
#include <flycapture/FlyCapture2.h>
#include <yaml-cpp/yaml.h>
#include "tmc_pgr_camera/i_point_grey_camera_system_setting.hpp"
#include "tmc_pgr_camera/point_grey_camera.hpp"

namespace {

// Maximum number of captured images to keep
const uint32_t kDefaultNumberOfImageBuffers = 2;
// Timeout period to obtain default image [ms]
const uint32_t kDefaultGrabImageTimeout = 10000;
// Time until RetrieveBuffer times out [ms]
const uint32_t kRetrieveTimeout = 800;
// Settings for continuous pulse output
const int kContinuouslyOutputPWMConfig = 0xFF;
// Log throttle time [ms]
const int kLogThrottleDuration = 5000;

/// @brief Convert RAW image to color or grayscale image
/// @param[in] image Input image
/// @param[in] image_type Specify color or monochrome
/// @return Converted image
/// @exception std::runtime_error In case of conversion failure
FlyCapture2::Image ConvertRawImage(const FlyCapture2::Image& image,
                                   const tmc_pgr_camera::ImageType image_type) {
  FlyCapture2::Image converted_image;
  const FlyCapture2::PixelFormat format = image.GetPixelFormat();
  if (format == FlyCapture2::PIXEL_FORMAT_RAW8 || format == FlyCapture2::PIXEL_FORMAT_RAW12 ||
      format == FlyCapture2::PIXEL_FORMAT_RAW16) {
    // Processing when camera output is RAW
    FlyCapture2::PixelFormat converted_format;
    if (image_type == tmc_pgr_camera::kRgbImage) {
      // Get RGB conversion destination pixel format
      switch (format) {
        case FlyCapture2::PIXEL_FORMAT_RAW8:
          converted_format = FlyCapture2::PIXEL_FORMAT_RGB8;
          break;
        case FlyCapture2::PIXEL_FORMAT_RAW12:
          // 12-bit RGB is not supported
          // Mono12 is set as it corresponds to incorrect flag settings
          converted_format = FlyCapture2::PIXEL_FORMAT_MONO12;
          break;
        default:
          converted_format = FlyCapture2::PIXEL_FORMAT_RGB16;
          break;
      }
    } else {
      // Get monochrome conversion destination pixel format
      switch (format) {
        case FlyCapture2::PIXEL_FORMAT_RAW8:
          converted_format = FlyCapture2::PIXEL_FORMAT_MONO8;
          break;
        case FlyCapture2::PIXEL_FORMAT_RAW12:
          converted_format = FlyCapture2::PIXEL_FORMAT_MONO12;
          break;
        default:
          converted_format = FlyCapture2::PIXEL_FORMAT_MONO16;
          break;
      }
    }
    // Image conversion
    const FlyCapture2::Error error = image.Convert(converted_format, &converted_image);
    if (error.GetType() != FlyCapture2::PGRERROR_OK) {
      throw std::runtime_error("Failed to convert image.");
    }
  }
  return converted_image;
}

/// @brief Convert FlyCapture2::Image to cv::Mat
/// @param[in] image Input image
/// @return Converted image
/// @exception std::runtime_error In case of conversion failure
cv::Mat ConvertFlyCaptureImageToCvMat(const FlyCapture2::Image& image) {
  cv::Mat cv_image;
  const cv::Size image_size(image.GetCols(), image.GetRows());
  switch (image.GetPixelFormat()) {
    case FlyCapture2::PIXEL_FORMAT_RAW8:
    case FlyCapture2::PIXEL_FORMAT_MONO8:
      cv_image = cv::Mat(image_size, CV_8UC1, image.GetData());
      break;
    case FlyCapture2::PIXEL_FORMAT_RAW16:
    case FlyCapture2::PIXEL_FORMAT_MONO16:
      cv_image = cv::Mat(image_size, CV_16UC1, image.GetData());
      break;
    case FlyCapture2::PIXEL_FORMAT_RGB8:
      cv_image = cv::Mat(image_size, CV_8UC3, image.GetData());
      break;
    default:
      throw std::runtime_error("Failed to convert image.");
  }

  return cv_image.clone();
}

/// @brief Obtain camera property array
/// @param[in] properties XML node describing camera properties
/// @return Property array
/// @exceptin In case of retrieval failure
std::vector<FlyCapture2::Property> GetCameraProperties(const YAML::Node& properties) {
  const std::string error_message = "Failed to set settings.";
  if (!properties.IsSequence()) {
    throw std::runtime_error(error_message + "\n'property' type is not array.");
  }

  std::unordered_map<std::string, FlyCapture2::PropertyType> types;
  types["brightness"] = FlyCapture2::BRIGHTNESS;
  types["auto_exposure"] = FlyCapture2::AUTO_EXPOSURE;
  types["sharpness"] = FlyCapture2::SHARPNESS;
  types["white_balance"] = FlyCapture2::WHITE_BALANCE;
  types["hue"] = FlyCapture2::HUE;
  types["saturation"] = FlyCapture2::SATURATION;
  types["gamma"] = FlyCapture2::GAMMA;
  types["iris"] = FlyCapture2::IRIS;
  types["focus"] = FlyCapture2::FOCUS;
  types["zoom"] = FlyCapture2::ZOOM;
  types["pan"] = FlyCapture2::PAN;
  types["tilt"] = FlyCapture2::TILT;
  types["shutter"] = FlyCapture2::SHUTTER;
  types["gain"] = FlyCapture2::GAIN;
  types["trigger_mode"] = FlyCapture2::TRIGGER_MODE;
  types["trigger_delay"] = FlyCapture2::TRIGGER_DELAY;
  types["frame_rate"] = FlyCapture2::FRAME_RATE;
  types["temperature"] = FlyCapture2::TEMPERATURE;

  std::vector<FlyCapture2::Property> flycap_properties;
  for (int32_t i = 0; i < properties.size(); ++i) {
    const YAML::Node& property_node = properties[i];
    if (!property_node.IsMap()) {
      throw std::runtime_error("'property' type is not struct.");
    }
    if (!property_node["type"]) {
      throw std::runtime_error("Not found key 'type' in property.");
    }
    std::string type = property_node["type"].as<std::string>();
    if (types.count(type) == 0) {
      throw std::runtime_error((boost::format("Invalid type.\ntype: %1%") % type).str());
    }

    FlyCapture2::Property property;
    property.type = types[type];

    if (property_node["present"]) {
      property.present = property_node["present"].as<bool>();
    }

    if (property_node["absControl"]) {
      property.absControl = property_node["absControl"].as<bool>();
    }

    if (property_node["onePush"]) {
      property.onePush = property_node["onePush"].as<bool>();
    }

    if (property_node["onOff"]) {
      property.onOff = property_node["onOff"].as<bool>();
    }

    if (property_node["autoManualMode"]) {
      property.autoManualMode = property_node["autoManualMode"].as<bool>();
    }

    if (property_node["valueA"]) {
      property.valueA = property_node["valueA"].as<uint32_t>();
    }

    if (property_node["valueB"]) {
      property.valueB = property_node["valueB"].as<uint32_t>();
    }

    if (property_node["absValue"]) {
      property.absValue = property_node["absValue"].as<float>();
    }

    if (property_node["reserved"]) {
      const YAML::Node& reserved = property_node["reserved"];
      if (!reserved.IsSequence()) {
        throw std::runtime_error("'reserved' type is not array.");
      }
      const uint32_t size = 8 < reserved.size() ? 8 : reserved.size();
      for (uint32_t i = 0; i < size; ++i) {
        property.reserved[i] = reserved[i].as<uint32_t>();
      }
    }

    flycap_properties.push_back(property);
  }

  return flycap_properties;
}

}  // anonymous namespace

namespace tmc_pgr_camera {

/// @brief Constructor
/// @param[in] camera_setting_file_path Path to camera setting file
PointGreyCameraSystem::PointGreyCameraSystem(std::shared_ptr<IPointGreyCameraSystemSetting>& camera_system_setting)
    : cameras_(),
      capture_thread_(),
      captured_images_(kDefaultNumberOfImageBuffers),
      camera_system_settings_(camera_system_setting),
      can_close_capture_thread_(false),
      access_() {
  Open();
}

/// @brief Destructor
PointGreyCameraSystem::~PointGreyCameraSystem() {
  try {
    Close();
  } catch (...) {
    // pass
  }
}

/// @brief Start camera system
/// @param[in] camera_setting_file_path Path to camera setting file
/// @exception std::runtime_error In case of settings failure
void PointGreyCameraSystem::Open() {
  if (IsOpened()) {
    Close();
  }

  // Generate camera
  const std::vector<uint32_t> serial_numbers = camera_system_settings_->GetSerialNumbers();
  for (const uint32_t serial_number : serial_numbers) {
    std::shared_ptr<PointGreyCamera> camera(new PointGreyCamera(serial_number));
    // Restart temporarily
    camera->RestartCamera();

    // Set video mode and frame rate
    const std::optional<FlyCapture2::VideoMode> video_mode = camera_system_settings_->GetVideoMode();
    if (video_mode) {
      const std::optional<std::pair<FlyCapture2::FrameRate, float> > frame_rate =
          camera_system_settings_->GetFrameRate(*video_mode);
      camera->SetVideoModeAndFrameRate(*video_mode, *frame_rate);
    }

    // Set properties
    camera->SetProperties(camera_system_settings_->GetProperties(), false);

    // Set RAW configuration for camera output
    const std::optional<FlyCapture2::Format7ImageSettings> format7_setting =
        camera_system_settings_->GetFormat7Setting();
    if (format7_setting) {
      camera->SetFormat7Configuration(*format7_setting);
    }

    cameras_.push_back(camera);
  }

  // Configure self-trigger
  if (camera_system_settings_->IsSelfTriggerEnabled()) {
    const std::optional<SelfTriggerSettings> self_trigger_setting = camera_system_settings_->GetSelfTriggerSettings();
    if (self_trigger_setting) {
      cameras_.front()->SetSelfTriggerSetting(self_trigger_setting->out_io, self_trigger_setting->pulse_figure);
    } else {
      throw std::runtime_error("Not found self trigger setting.");
    }
    // Confirm 3.3V output setting and set if necessary
    const std::optional<bool> output_voltage_enable = camera_system_settings_->GetOutputVoltageSetting();
    if (output_voltage_enable) {
      if (*output_voltage_enable) {
        cameras_.front()->OutputVoltage(true);
      }
    }
  }

  std::optional<FlyCapture2::ColorProcessingAlgorithm> software_demosaicing =
      camera_system_settings_->GetSoftDemosaicing();
  if (!software_demosaicing) {
    software_demosaicing = FlyCapture2::DEFAULT;
    CONSOLE_BRIDGE_logWarn("Use default software demosaicing setting.");
  }

  FlyCapture2::Error error = FlyCapture2::Image::SetDefaultColorProcessing(*software_demosaicing);
  if (error.GetType() != FlyCapture2::PGRERROR_OK) {
    throw std::runtime_error("Failed to set software demosaicing.");
  }
}

/// @brief Terminate camera system
void PointGreyCameraSystem::Close() {
  if (IsCapturing()) {
    StopCapture();
  }
  cameras_.clear();
}

/// @brief Start capture
/// @exception std::runtime_error In case of start failure
void PointGreyCameraSystem::StartCapture() {
  if (IsCapturing()) {
    CONSOLE_BRIDGE_logWarn("Already start capture.");
    return;
  }

  if (cameras_.empty()) {
    throw std::runtime_error("There are no cameras.");
  }

  capture_thread_ = std::thread(&PointGreyCameraSystem::CaptureThread, this);
  // Wait for thread start
  uint32_t wait_count = 0;
  while (true) {
    if (IsCapturing()) {
      break;
    }
    // If not started after 100ms, consider it abnormal
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ++wait_count;
    if (wait_count == 100) {
      throw std::runtime_error("Failed to start capture with timeout.\n");
    }
  }
  // Countermeasures for abnormalities only in initial image acquisition with trigger mode
  const std::optional<FlyCapture2::TriggerMode> trigger_mode = camera_system_settings_->GetTriggerMode();
  if (trigger_mode && trigger_mode->onOff) {
    // Trigger signal issuance for the first time only in software and self-trigger modes
    // To avoid blocking in the first call of RetrieveBuffer within the image acquisition thread function
    // This resolves the issue of acquiring the first image without external trigger input
    // It's better to review the relationship between the main thread and the image acquisition thread
    if (camera_system_settings_->IsSoftwareTriggerEnabled()) {
      // First shot for software trigger
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      cameras_.front()->SetSoftwareTrigger(true);
    } else if (camera_system_settings_->IsSelfTriggerEnabled()) {
      // First shot for self-trigger
      // Employ wait as it doesn't work immediately
      // To advance processing within capture thread (★important)
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      const std::optional<SelfTriggerSettings> self_trigger_setting =
          camera_system_settings_->GetSelfTriggerSettings();
      if (self_trigger_setting) {
        cameras_.front()->SendPwmForSelfTrigger(self_trigger_setting->out_io, self_trigger_setting->number_of_pulse,
                                                self_trigger_setting->polarity);
      } else {
        throw std::runtime_error("Not found self trigger setting.");
      }
    }
  }
}

/// @brief Stop capture
/// @exception std::runtime_error Capture stop failure
void PointGreyCameraSystem::StopCapture() {
  // Terminate capture thread
  {
    std::unique_lock<std::shared_mutex> write(access_);
    can_close_capture_thread_ = true;
  }

  if (cameras_.empty()) {
    throw std::runtime_error("There are no cameras.");
  }

  // Termination process in trigger mode
  if (cameras_.front()->GetTriggerMode().onOff) {
    // Drive last capture as it's on standby in image acquisition processing thread
    // In case of external trigger timeout for IEEE1394 camera,
    // Apply software trigger to avoid image acquisition lock in thread
    // However, it has no effect on USB cameras
    // For software trigger

    // NOTE: Humble crashes the moment the function is called
    // cameras_.front()->SetSoftwareTrigger(true);

    if (camera_system_settings_->IsSelfTriggerEnabled()) {
      // Send PWM waveform for self-trigger
      const std::optional<SelfTriggerSettings> self_trigger_setting =
          camera_system_settings_->GetSelfTriggerSettings();
      if (self_trigger_setting) {
        // Issue self-trigger pulse if finite
        if (self_trigger_setting->number_of_pulse != kContinuouslyOutputPWMConfig) {
          cameras_.front()->SendPwmForSelfTrigger(
              self_trigger_setting->out_io,
              self_trigger_setting->number_of_pulse,
              self_trigger_setting->polarity);
        }
      } else {
        throw std::runtime_error("Not found self trigger setting.");
      }
    }
  }

  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }

  // Termination process in trigger mode
  typedef std::shared_ptr<PointGreyCamera> CameraPtr;
  if (cameras_.front()->GetTriggerMode().onOff) {
    // NOTE: Humble crashes the moment the function is called
    // cameras_.front()->SetSoftwareTrigger(false);
    for (const CameraPtr& camera : cameras_) { camera->SetTriggerMode(FlyCapture2::TriggerMode()); }
  }

  // Stop PWM waveform emission for self-trigger
  if (camera_system_settings_->IsSelfTriggerEnabled()) {
    const std::optional<SelfTriggerSettings> self_trigger_setting = camera_system_settings_->GetSelfTriggerSettings();
    if (self_trigger_setting) {
      cameras_.front()->StopPwmForSelfTrigger(self_trigger_setting->out_io);
    } else {
      throw std::runtime_error("Not found self trigger setting.");
    }
  }

  // Stop if capturing
  for (const CameraPtr& camera : cameras_) {
    if (camera->is_capturing()) {
      camera->StopCapture();
    }
  }
}

/// @brief Obtain capture image
/// @return Array of acquired capture images (for number of cameras)
/// @exception std::runtime_error In case of capture image acquisition failure
std::optional<std::vector<ImagePtr> > PointGreyCameraSystem::GrabImage() {
  if (cameras_.empty()) {
    throw std::runtime_error("Failed to grab image.");
  }

  // Image acquisition wait processing in external, software, and self-trigger modes
  std::optional<std::chrono::system_clock::time_point> time_stamp;
  if (cameras_.front()->GetTriggerMode().onOff) {
    // If trigger is ON, set data to the time before Trigger call
    time_stamp = std::chrono::system_clock::now();
    if (camera_system_settings_->IsSoftwareTriggerEnabled()) {
      // Start software trigger mode
      if (!cameras_.front()->SetSoftwareTrigger(true)) {
        return std::nullopt;
      }
    } else if (camera_system_settings_->IsSelfTriggerEnabled()) {
      // Send PWM waveform for self-trigger
      const std::optional<SelfTriggerSettings> self_trigger_setting =
          camera_system_settings_->GetSelfTriggerSettings();
      if (self_trigger_setting) {
        // Issue self-trigger pulse if finite
        if (self_trigger_setting->number_of_pulse != kContinuouslyOutputPWMConfig) {
          cameras_.front()->SendPwmForSelfTrigger(
              self_trigger_setting->out_io,
              self_trigger_setting->number_of_pulse,
              self_trigger_setting->polarity);
        }
      } else {
        throw std::runtime_error("Not found self trigger setting.");
      }
    }
  }

  // Image acquisition processing wait
  uint32_t timeout = 0;
  std::optional<std::vector<ImagePtr> > images;
  while (true) {
    {
      // NOTE: No equivalent mechanism to boost::upgrade_lock exists in std.
      // Assumed no deadlock even if locked from the start, rather than Read-Only -> Lock procedure
      // Apply unique_lock to this entire block.
      std::unique_lock<std::shared_mutex> write_lock(access_);
      // boost::upgrade_lock<boost::shared_mutex> upgrade_lock(access_);
      if (!captured_images_.empty()) {
        images = captured_images_.back();
        // boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);
        captured_images_.pop_back();
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ++timeout;
    if (kDefaultGrabImageTimeout < timeout) {
      throw std::runtime_error("Time out retrive flag on.");
    }
  }

  // If trigger is ON, set data to the time before Trigger call
  if (time_stamp) {
    for (const ImagePtr& image : *images) { image->time = *time_stamp; }
  }

  return images;
}

/// @brief Check if camera system is running
/// @return Returns true if there is at least one camera, and all are connected
bool PointGreyCameraSystem::IsOpened() const {
  bool is_opened = true;
  typedef std::shared_ptr<PointGreyCamera> CameraPtr;
  is_opened &= 0 < cameras_.size();
  for (const CameraPtr& camera : cameras_) {
    if (!camera) {
      continue;
    }
    is_opened &= camera->IsOpened();
  }
  return is_opened;
}

/// @brief Check if capturing
/// @return Returns true if capture is occurring
bool PointGreyCameraSystem::IsCapturing() const {
  bool is_capturing = true;
  typedef std::shared_ptr<PointGreyCamera> CameraPtr;
  is_capturing &= 0 < cameras_.size();
  for (const CameraPtr& camera : cameras_) {
    if (!camera) {
      continue;
    }
    is_capturing &= camera->is_capturing();
  }
  return is_capturing;
}

/// @brief Camera settings
/// @param[in] settings Node describing settings
/// @exception std::runtime_error In case of setting failure
/// @note The values that can be set are as follows
///       ・FlyCapture2::Property
///         Example:
///         {
///           property: [
///             { type: brightness, absValue: 0.0, ..., onePush: on },
///             { type: saturation, ... }
///           ]
///         }
///
///         Settable types are brightness, auto_exposure, sharpness, white_balance,
///         hue、saturation、gamma、iris、focus、zoom、pan、tilt、shutter、gain、
///         trigger_mode、trigger_delay、frame_rate、temperature
///         Settable values are present: bool, absControl: bool, onePush: bool,
///         onOff: bool、autoManualMode: bool、valueA: unsigned int、
///         valueB: unsigned int、absValue: float、reserved: unsigned int[8]
void PointGreyCameraSystem::SetSettings(const YAML::Node& settings) {
  if (!IsOpened()) {
    throw std::runtime_error("Camera system is not running.");
  }

  const std::string error_message = "Failed to set settings.";
  if (!settings.IsMap()) {
    throw std::runtime_error(error_message + "\n'setting' type is not struct.");
  }

  // Setting properties
  if (settings["property"]) {
    const std::vector<FlyCapture2::Property>& properties = GetCameraProperties(settings["property"]);
    typedef std::shared_ptr<PointGreyCamera> CameraPtr;
    bool show_result = true;
    for (const CameraPtr& camera : cameras_) {
      camera->SetProperties(properties, show_result);
      // Display setting results of properties only for the first camera
      show_result = false;
    }
    // Setting trigger mode
    for (int32_t i = 0; i < settings["property"].size(); ++i) {
      const YAML::Node& property_node = settings["property"][i];
      std::string type = property_node["type"].as<std::string>();
      if (type == "trigger_mode") {
        int32_t trigger_mode_mode = 0;
        if (property_node["mode"]) {
          trigger_mode_mode = static_cast<int32_t>(property_node["mode"].as<int64_t>());
        }
        bool trigger_mode_on_off = false;
        if (property_node["onOff"]) {
          trigger_mode_on_off = property_node["onOff"].as<bool>();
        }
        int32_t trigger_mode_polarity = 0;
        if (property_node["polarity"]) {
          trigger_mode_polarity = static_cast<int32_t>(property_node["polarity"].as<int64_t>());
        }
        camera_system_settings_->UpdateTriggerMode(trigger_mode_mode, trigger_mode_on_off, trigger_mode_polarity);
        for (const CameraPtr& camera : cameras_) {
          const std::optional<FlyCapture2::TriggerMode> trigger_mode = camera_system_settings_->GetTriggerMode();
          camera->SetTriggerMode(*trigger_mode);
        }
        break;
      }
    }
  } else {
    throw std::runtime_error(error_message);
  }
}

/// @brief Capture thread
/// @exception std::runtime_error In case of processing failure during capture
void PointGreyCameraSystem::CaptureThread() {
  if (cameras_.empty()) {
    throw std::runtime_error("Failed to capture.");
  }
  // Setting trigger mode and delay
  typedef std::shared_ptr<PointGreyCamera> CameraPtr;
  for (const CameraPtr& camera : cameras_) {
    const std::optional<FlyCapture2::TriggerMode> trigger_mode = camera_system_settings_->GetTriggerMode();
    if (trigger_mode) {
      camera->SetTriggerMode(*trigger_mode);
    }
    const std::optional<FlyCapture2::TriggerDelay> trigger_delay = camera_system_settings_->GetTriggerDelay();
    if (trigger_delay) {
      camera->SetTriggerDelay(*trigger_delay);
    }
  }

  // Switch capture start API between trigger mode and free-running mode
  const FlyCapture2::TriggerMode trigger_mode = cameras_.front()->GetTriggerMode();
  if (trigger_mode.onOff) {
    for (const CameraPtr& camera : cameras_) {
      // Set timeout duration for RetrieveBuffer
      FlyCapture2::FC2Config config;
      config.grabTimeout = kRetrieveTimeout;
      camera->SetConfiguration(config);

      // Start capture
      camera->StartCapture();
    }
  } else {
    // Start capture
    PointGreyCamera::StartSyncCapture(cameras_);
  }

  // Turn off thread stop command
  {
    std::unique_lock<std::shared_mutex> write(access_);
    can_close_capture_thread_ = false;
  }
  // Periodic processing
  std::optional<ImageType> image_type = camera_system_settings_->GetImageType();
  if (!image_type) {
    image_type = kMonoImage;
  }

  // For log throttling
  std::string last_message;
  std::chrono::system_clock::time_point last_time;
  while (true) {
    try {
      std::vector<ImagePtr> images;
      for (const CameraPtr& camera : cameras_) {
        std::this_thread::yield();
        // Data acquisition
        const FlyCapture2::Image image = ConvertRawImage(camera->RetrieveBuffer(), *image_type);
        ImagePtr output_image(new Image());
        output_image->image = ConvertFlyCaptureImageToCvMat(image),
        output_image->time = std::chrono::system_clock::now();
        images.push_back(output_image);
      }

      {
        std::unique_lock<std::shared_mutex> write(access_);
        // Insert image into buffer
        captured_images_.push_back(images);
      }
    } catch (const std::runtime_error& e) {
      // Apply throttle to log
      // Output to log only for new messages or after specified time has elapsed
      std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
      std::string message(e.what());
      if (last_message != message ||
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count() > kLogThrottleDuration) {
        last_time = now;
        last_message = message;
        CONSOLE_BRIDGE_logWarn("%s", message.c_str());
      }
    }

    // Check thread stop command, if on terminate the thread
    std::shared_lock<std::shared_mutex> read(access_);
    if (can_close_capture_thread_) {
      break;
    }
  }
}
}  // end of namespace tmc_pgr_camera
