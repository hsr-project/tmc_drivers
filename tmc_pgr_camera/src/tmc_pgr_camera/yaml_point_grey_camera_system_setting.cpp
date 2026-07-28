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
/// @brief      Load the configuration file for the Point Grey camera system
#include "tmc_pgr_camera/yaml_point_grey_camera_system_setting.hpp"
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <boost/format.hpp>
#include <wordexp.h>

namespace {

// Parameter value for software trigger in trigger mode settings
const uint32_t kSoftwareTriggerParameter = 1;

// Source value for software trigger in trigger mode settings
const uint32_t kSoftwareTriggerSource = 7;
}

namespace YAML {

/// @brief Function object to convert YAML::Node to FlyCapture2::Property
template <>
struct convert<FlyCapture2::Property> {
  /// @brief Convert from YAML::Node to FlyCapture2::Property
  /// @param[in] node YAML node
  /// @param[in] property Camera property of FlyCapture2 API
  /// @return
  /// Returns true if the number of properties in the node matches the number of properties successfully set
  static bool decode(const Node& node, FlyCapture2::Property& property) {
    uint32_t count = 0;
    if (node["type"]) {
      property.type = static_cast<FlyCapture2::PropertyType>(node["type"].as<int32_t>());
      ++count;
    }
    if (node["present"]) {
      property.present = node["present"].as<bool>();
      ++count;
    }
    if (node["absControl"]) {
      property.absControl = node["absControl"].as<bool>();
      ++count;
    }
    if (node["onePush"]) {
      property.onePush = node["onePush"].as<bool>();
      ++count;
    }
    if (node["onOff"]) {
      property.onOff = node["onOff"].as<bool>();
      ++count;
    }
    if (node["autoManualMode"]) {
      property.autoManualMode = node["autoManualMode"].as<bool>();
      ++count;
    }
    if (node["valueA"]) {
      property.valueA = node["valueA"].as<uint32_t>();
      ++count;
    }
    if (node["valueB"]) {
      property.valueB = node["valueB"].as<uint32_t>();
      ++count;
    }
    if (node["absValue"]) {
      property.absValue = node["absValue"].as<float>();
      ++count;
    }
    const Node& reserved = node["reserved"];
    if (reserved && reserved.IsSequence()) {
      const size_t size = 8 < reserved.size() ? 8 : reserved.size();
      for (size_t i = 0; i < size; ++i) {
        property.reserved[i] = reserved[i].as<uint32_t>();
      }
      ++count;
    }
    return count == node.size();
  }
};

/// @brief Function object to convert YAML::Node to FlyCapture2::TriggerMode
template <>
struct convert<FlyCapture2::TriggerMode> {
  /// @brief Convert from YAML::Node to FlyCapture2::TriggerMode
  /// @param[in] node YAML node
  /// @param[in] property Trigger mode of FlyCapture2 API
  /// @return
  /// Returns true if the number of properties in the node matches the number of properties successfully set
  static bool decode(const Node& node, FlyCapture2::TriggerMode& trigger_mode) {
    uint32_t count = 0;
    if (node["onOff"]) {
      trigger_mode.onOff = node["onOff"].as<bool>();
      ++count;
    }
    if (node["polarity"]) {
      trigger_mode.polarity = node["polarity"].as<uint32_t>();
      ++count;
    }
    if (node["source"]) {
      trigger_mode.source = node["source"].as<uint32_t>();
      ++count;
    }
    if (node["mode"]) {
      trigger_mode.mode = node["mode"].as<uint32_t>();
      ++count;
    }
    if (node["parameter"]) {
      trigger_mode.parameter = node["parameter"].as<uint32_t>();
      ++count;
    }
    const Node& reserved = node["reserved"];
    if (reserved && reserved.IsSequence()) {
      const size_t size = 8 < reserved.size() ? 8 : reserved.size();
      for (size_t i = 0; i < size; ++i) {
        trigger_mode.reserved[i] = reserved[i].as<uint32_t>();
      }
      ++count;
    }
    return count == node.size();
  }
};

}  // end of namespace YAML

namespace tmc_pgr_camera {

/// @brief Constructor
/// @param[in] camera_setting_file_path Path to the camera setting file
/// @exception std::runtime_error Thrown if the file cannot be loaded
/// @exception std::runtime_error Thrown if the node is invalid
YamlPointGreyCameraSystemSetting::YamlPointGreyCameraSystemSetting(const std::string& camera_setting_file_path)
    : setting_node_(), cache_() {
  try {
    // Expand ~ using $HOME
    wordexp_t expanded_result;
    ::wordexp(camera_setting_file_path.c_str(), &expanded_result, 0);
    const std::string camera_setting_file_full_path(expanded_result.we_wordv[0]);
    ::wordfree(&expanded_result);
    setting_node_ = YAML::LoadFile(camera_setting_file_full_path);
  } catch (const YAML::Exception& e) {
    std::string description = "Failed to read file. File: " + camera_setting_file_path + "\nDetail: ";
    description = description + e.what();
    throw std::runtime_error(description);
  }
}

/// @brief Retrieve the array of serial numbers for cameras used in the camera system
/// @return Array of camera serial numbers
///         The master camera is listed first
/// @exception std::runtime_error Thrown if camera information retrieval fails
/// @exception std::runtime_error Thrown if camera object creation fails
/// @exception std::runtime_error Thrown if the node is invalid
/// @note Information required for camera details is in the following YAML format (id is the camera's serial number)
///       If there are two or more cameras, setting master: true designates that camera as the master
///       (If multiple master settings exist, the first master is adopted)
///       cameras:
///         - id: xxxxxxxx
///           master: true
///         - id: yyyyyyyy
std::vector<uint32_t> YamlPointGreyCameraSystemSetting::GetSerialNumbers() {
  const int32_t hash = __LINE__;
  if (cache_.count(hash)) {
    return std::any_cast<std::vector<uint32_t> >(cache_[hash]);
  }
  const YAML::Node& camera_settings = setting_node_["cameras"];
  if (!camera_settings || !camera_settings.IsSequence()) {
    throw std::runtime_error("Not found key 'cameras' or 'cameras' node type is not sequence.");
  }
  std::vector<uint32_t> serial_numbers;
  std::optional<uint32_t> master_serial_number;
  for (const YAML::Node& camera_setting : camera_settings) {
    const YAML::Node& id = camera_setting["id"];
    if (!id || !id.IsScalar()) {
      throw std::runtime_error(
          "Not found serial number of camera or "
          "serial number node type is not scalar.");
    }
    const uint32_t serial_number = id.as<uint32_t>();

    const YAML::Node& is_master = camera_setting["master"];
    try {
      if (is_master && is_master.as<bool>() && !master_serial_number) {
        master_serial_number = serial_number;
      } else {
        serial_numbers.push_back(serial_number);
      }
    } catch (const YAML::Exception& e) {
      std::string description = "Set 'master' of camera setting to boolean value in configuration YAML file.\nDetail: ";
      description = description + e.what();
      throw std::runtime_error(description);
    }
  }

  if (serial_numbers.empty() && !master_serial_number) {
    throw std::runtime_error(
        "There are no camera settings "
        "in configuration YAML file.");
  }

  if (master_serial_number) {
    serial_numbers.insert(serial_numbers.begin(), *master_serial_number);
  }

  cache_[hash] = serial_numbers;

  return serial_numbers;
}

/// @brief Load the array of camera properties
/// @return Array of camera properties
/// @exception std::runtime_error Thrown if a non-existent property is specified
/// @exception std::runtime_error Thrown if the node is invalid
/// @note Information required for camera properties is in the following YAML format
///       Example:
///       parameter:
///         properties:
///           brightness: { absValue: 0.0, ... }
///           saturation: { ... }
///           zoom: { ... }
///
///       Configurable keys include brightness, auto_exposure, sharpness, white_balance,
///       hue、saturation、gamma、iris、focus、zoom、pan、tilt、shutter、gain、
///       trigger_mode、trigger_delay、frame_rate、temperature
///       Configurable values include present: bool, absControl: bool, onePush: bool,
///       onOff: bool、autoManualMode: bool、valueA: unsigned int、
///       valueB: unsigned int、absValue: float、reserved: unsigned int[8]
std::vector<FlyCapture2::Property> YamlPointGreyCameraSystemSetting::GetProperties() {
  const int32_t hash = __LINE__;
  if (cache_.count(hash)) {
    return std::any_cast<std::vector<FlyCapture2::Property> >(cache_[hash]);
  }
  std::vector<FlyCapture2::Property> camera_properties;
  const YAML::Node& parameter = setting_node_["parameter"];
  if (!parameter || !parameter.IsMap()) {
    return camera_properties;
  }
  const YAML::Node& properties = parameter["properties"];
  if (!properties || !parameter.IsMap()) {
    return camera_properties;
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

  try {
    for (YAML::const_iterator node = properties.begin(); node != properties.end(); ++node) {
      const std::string key = node->first.as<std::string>();
      if (types.count(key) == 0) {
        const std::string msg = "'%1%' does not exist in camera properties.";
        throw std::runtime_error((boost::format(msg) % key).str());
      }
      FlyCapture2::Property property = node->second.as<FlyCapture2::Property>();
      property.type = types[key];
      camera_properties.push_back(property);
    }
  } catch (const YAML::Exception& e) {
    std::string description = "Failed to get camera properties.\nDetail: ";
    description = description + e.what();
    throw std::runtime_error(description);
  }

  cache_[hash] = camera_properties;

  return camera_properties;
}

/// @brief Retrieve the frame rate
/// @param[in] video_mode Video format to be used
/// @return Pair of the FlyCapture2 SDK constant value for the retrieved frame rate and
///         the actual frame rate value. For Format7, the constant value is FRAMERATE_FORMAT7
///         Returns an invalid value if no setting exists
/// @exception std::runtime_error Thrown if a non-existent frame rate is set
/// @exception std::runtime_error Thrown if the node is invalid
/// @note Information required for frame rate is in the following YAML format
///       Example:
///       parameter:
///         frame_rate: 7.5
///
///       If not Format7, frame_rate can be set to one of the following:
///       1.875, 3.75, 7.5, 15, 30, 60, 120, 240
std::optional<std::pair<FlyCapture2::FrameRate, float> > YamlPointGreyCameraSystemSetting::GetFrameRate(
    const FlyCapture2::VideoMode video_mode) {
  const YAML::Node& parameter = setting_node_["parameter"];
  if (!parameter || !parameter.IsMap()) {
    return std::nullopt;
  }
  const YAML::Node& frame_rate = parameter["frame_rate"];
  if (!frame_rate) {
    return std::nullopt;
  }

  std::optional<std::pair<FlyCapture2::FrameRate, float> > frame_rate_opt;
  try {
    const float fps = frame_rate.as<float>();
    if (video_mode == FlyCapture2::VIDEOMODE_FORMAT7) {
      return std::make_pair(FlyCapture2::FRAMERATE_FORMAT7, fps);
    }

    std::unordered_map<float, FlyCapture2::FrameRate> frame_rates;
    frame_rates[1.875] = FlyCapture2::FRAMERATE_1_875;
    frame_rates[3.75] = FlyCapture2::FRAMERATE_3_75;
    frame_rates[7.5] = FlyCapture2::FRAMERATE_7_5;
    frame_rates[15] = FlyCapture2::FRAMERATE_15;
    frame_rates[30] = FlyCapture2::FRAMERATE_30;
    frame_rates[60] = FlyCapture2::FRAMERATE_60;
    frame_rates[120] = FlyCapture2::FRAMERATE_120;
    frame_rates[240] = FlyCapture2::FRAMERATE_240;
    if (frame_rates.count(fps) == 0) {
      throw std::runtime_error((boost::format("Invalid frame rate.\nvalue: %1%") % fps).str());
    }

    frame_rate_opt = std::make_pair(frame_rates[fps], fps);
  } catch (const YAML::Exception& e) {
    std::string description = "Failed to get camera frame rate.\nDetail: ";
    description = description + e.what();
    throw std::runtime_error(description);
  }

  return frame_rate_opt;
}

/// @brief Retrieve the video mode
/// @return FlyCapture2 SDK constant value for the retrieved video mode
///         Returns an invalid value if no setting exists
/// @exception std::runtime_error Thrown if a non-existent video mode is specified
/// @exception std::runtime_error Thrown if the node is invalid
/// @note Information required for video mode is in the following YAML format
///       Example:
///       parameter:
///         video_mode: "format7"
///
///       Configurable values include 160x120yuv444, 1280x960rgb, 1024x768rgb, 1024x768y16,
///       640x480y8、1280x960y8、800x600y16、1600x1200y16、320x240yuv422、
///       1024x768y8、800x600y8、640x480yuv411、1600x1200rgb、1280x960yuv422、
///       1600x1200yuv422、640x480yuv422、640x480y16、1280x960y16、1600x1200y8、
///       640x480rgb、800x600rgb、800x600yuv422、1024x768yuv422、format7
std::optional<FlyCapture2::VideoMode> YamlPointGreyCameraSystemSetting::GetVideoMode() {
  const int32_t hash = __LINE__;
  if (cache_.count(hash)) {
    return std::any_cast<FlyCapture2::VideoMode>(cache_[hash]);
  }
  const YAML::Node& parameter = setting_node_["parameter"];
  if (!parameter || !parameter.IsMap()) {
    return std::nullopt;
  }
  const YAML::Node& video_mode = parameter["video_mode"];
  if (!video_mode) {
    return std::nullopt;
  }
  std::unordered_map<std::string, FlyCapture2::VideoMode> video_modes;
  video_modes["160x120yuv444"] = FlyCapture2::VIDEOMODE_160x120YUV444;
  video_modes["320x240yuv422"] = FlyCapture2::VIDEOMODE_320x240YUV422;
  video_modes["640x480yuv411"] = FlyCapture2::VIDEOMODE_640x480YUV411;
  video_modes["640x480yuv422"] = FlyCapture2::VIDEOMODE_640x480YUV422;
  video_modes["640x480rgb"] = FlyCapture2::VIDEOMODE_640x480RGB;
  video_modes["640x480y8"] = FlyCapture2::VIDEOMODE_640x480Y8;
  video_modes["640x480y16"] = FlyCapture2::VIDEOMODE_640x480Y16;
  video_modes["800x600yuv422"] = FlyCapture2::VIDEOMODE_800x600YUV422;
  video_modes["800x600rgb"] = FlyCapture2::VIDEOMODE_800x600RGB;
  video_modes["800x600y8"] = FlyCapture2::VIDEOMODE_800x600Y8;
  video_modes["800x600y16"] = FlyCapture2::VIDEOMODE_800x600Y16;
  video_modes["1024x768yuv422"] = FlyCapture2::VIDEOMODE_1024x768YUV422;
  video_modes["1024x768rgb"] = FlyCapture2::VIDEOMODE_1024x768RGB;
  video_modes["1024x768y8"] = FlyCapture2::VIDEOMODE_1024x768Y8;
  video_modes["1024x768y16"] = FlyCapture2::VIDEOMODE_1024x768Y16;
  video_modes["1280x960yuv422"] = FlyCapture2::VIDEOMODE_1280x960YUV422;
  video_modes["1280x960rgb"] = FlyCapture2::VIDEOMODE_1280x960RGB;
  video_modes["1280x960y8"] = FlyCapture2::VIDEOMODE_1280x960Y8;
  video_modes["1280x960y16"] = FlyCapture2::VIDEOMODE_1280x960Y16;
  video_modes["1600x1200yuv422"] = FlyCapture2::VIDEOMODE_1600x1200YUV422;
  video_modes["1600x1200rgb"] = FlyCapture2::VIDEOMODE_1600x1200RGB;
  video_modes["1600x1200y8"] = FlyCapture2::VIDEOMODE_1600x1200Y8;
  video_modes["1600x1200y16"] = FlyCapture2::VIDEOMODE_1600x1200Y16;
  video_modes["format7"] = FlyCapture2::VIDEOMODE_FORMAT7;

  std::optional<FlyCapture2::VideoMode> video_mode_opt;
  try {
    const std::string key = video_mode.as<std::string>();
    if (video_modes.count(key) == 0) {
      throw std::runtime_error((boost::format("Invalid video mode.\nmode: %1%") % key).str());
    }
    video_mode_opt = video_modes[key];
    cache_[hash] = *video_mode_opt;
  } catch (const YAML::Exception& e) {
    std::string description = "Failed to get camera video mode.\nDetail: ";
    description = description + e.what();
    throw std::runtime_error(description);
  }

  return video_mode_opt;
}

/// @brief Retrieve Format7 settings
/// @return Retrieved Format7 settings
///         Returns an invalid value if no setting exists
/// @exception std::runtime_error Thrown if the node key does not exist
/// @exception std::runtime_error Thrown if the node is invalid
/// @note Information required for Format7 settings is in the following YAML format
///       Example:
///       parameter:
///　　　　  format7:
///　　　　    mode: 0
///　　　　    offsetX: 8
///　　　　    offsetY: 2
///　　　　    Width: 1280
///　　　　    Height: 960
///　　　　    pixel_format: "raw8"
///
///       mode is the Format7 mode and can be set to a value between 0 and 31
///       Additionally, Offset is set to half the difference between the actual camera resolution and the configured width and height
///       Configurable pixel_format values include mono8, raw8, s_rgb16, mono12, rgb8, raw16,
///       s_mono16、422yuv8、rgb、411yuv8、raw12、mono16、bgr、bgru、rgb16、
///       444yuv8、rgbu、bgr16、bgru16、422yuv8_jpeg
std::optional<FlyCapture2::Format7ImageSettings> YamlPointGreyCameraSystemSetting::GetFormat7Setting() {
  const int32_t hash = __LINE__;
  if (cache_.count(hash)) {
    return std::any_cast<FlyCapture2::Format7ImageSettings>(cache_[hash]);
  }
  const YAML::Node& parameter = setting_node_["parameter"];
  if (!parameter || !parameter.IsMap()) {
    return std::nullopt;
  }
  const YAML::Node& format7_setting = parameter["format7"];
  if (!format7_setting) {
    return std::nullopt;
  }

  std::unordered_map<std::string, FlyCapture2::PixelFormat> pixel_formats;
  pixel_formats["mono8"] = FlyCapture2::PIXEL_FORMAT_MONO8;
  pixel_formats["raw8"] = FlyCapture2::PIXEL_FORMAT_RAW8;
  pixel_formats["s_rgb16"] = FlyCapture2::PIXEL_FORMAT_S_RGB16;
  pixel_formats["mono12"] = FlyCapture2::PIXEL_FORMAT_MONO12;
  pixel_formats["rgb8"] = FlyCapture2::PIXEL_FORMAT_RGB8;
  pixel_formats["raw16"] = FlyCapture2::PIXEL_FORMAT_RAW16;
  pixel_formats["s_mono16"] = FlyCapture2::PIXEL_FORMAT_S_MONO16;
  pixel_formats["422yuv8"] = FlyCapture2::PIXEL_FORMAT_422YUV8;
  pixel_formats["rgb"] = FlyCapture2::PIXEL_FORMAT_RGB;
  pixel_formats["411yuv8"] = FlyCapture2::PIXEL_FORMAT_411YUV8;
  pixel_formats["raw12"] = FlyCapture2::PIXEL_FORMAT_RAW12;
  pixel_formats["mono16"] = FlyCapture2::PIXEL_FORMAT_MONO16;
  pixel_formats["bgr"] = FlyCapture2::PIXEL_FORMAT_BGR;
  pixel_formats["bgru"] = FlyCapture2::PIXEL_FORMAT_BGRU;
  pixel_formats["rgb16"] = FlyCapture2::PIXEL_FORMAT_RGB16;
  pixel_formats["444yuv8"] = FlyCapture2::PIXEL_FORMAT_444YUV8;
  pixel_formats["rgbu"] = FlyCapture2::PIXEL_FORMAT_RGBU;
  pixel_formats["bgr16"] = FlyCapture2::PIXEL_FORMAT_BGR16;
  pixel_formats["bgru16"] = FlyCapture2::PIXEL_FORMAT_BGRU16;
  pixel_formats["422yuv8_jpeg"] = FlyCapture2::PIXEL_FORMAT_422YUV8_JPEG;

  std::optional<FlyCapture2::Format7ImageSettings> format7_image_settings_opt;
  try {
    FlyCapture2::Format7ImageSettings format7_image_settings;

    const YAML::Node& mode = format7_setting["mode"];
    if (!mode) {
      throw std::runtime_error("Not found key 'mode' in format7 setting.");
    }
    const uint32_t mode_number = mode.as<uint32_t>();
    if (31 < mode_number) {
      throw std::runtime_error(
          "Invalid mode of format7 in configuration YAML file.\n"
          "Set mode from 0 to 31.");
    }
    format7_image_settings.mode = static_cast<FlyCapture2::Mode>(mode_number);
    const YAML::Node& width = format7_setting["width"];
    if (!width) {
      throw std::runtime_error("Not found key 'width' in format7 setting.");
    }
    format7_image_settings.width = width.as<uint32_t>();
    const YAML::Node& height = format7_setting["height"];
    if (!height) {
      throw std::runtime_error("Not found key 'height' in format7 setting.");
    }
    format7_image_settings.height = height.as<uint32_t>();
    const YAML::Node& offset_x = format7_setting["offsetX"];
    if (!offset_x) {
      throw std::runtime_error("Not found key 'offsetX' in format7 setting.");
    }
    format7_image_settings.offsetX = offset_x.as<uint32_t>();
    const YAML::Node& offset_y = format7_setting["offsetY"];
    if (!offset_y) {
      throw std::runtime_error("Not found key 'offsetY' in format7 setting.");
    }
    format7_image_settings.offsetY = offset_y.as<uint32_t>();
    const YAML::Node& pixel_format = format7_setting["pixelFormat"];
    if (!pixel_format) {
      throw std::runtime_error("Not found key 'pixelFormat' in format7 setting.");
    }
    if (pixel_formats.count(pixel_format.as<std::string>()) == 0) {
      throw std::runtime_error((boost::format("Unknown pixel format '%1%'.") % pixel_format).str());
    }
    format7_image_settings.pixelFormat = pixel_formats[pixel_format.as<std::string>()];
    format7_image_settings_opt = format7_image_settings;
    cache_[hash] = *format7_image_settings_opt;
  } catch (const YAML::Exception& e) {
    std::string description = "Failed to get format7 setting.\nDetail: ";
    description = description + e.what();
    throw std::runtime_error(description);
  }

  return format7_image_settings_opt;
}

/// @brief Retrieve software demosaicing settings
/// @return Retrieved software demosaicing settings
///         Returns an invalid value if no setting exists
/// @exception std::runtime_error Thrown if an incorrect string is specified for the setting
/// @exception std::runtime_error Thrown if the node is invalid
/// @note Information required for software demosaicing settings is in the following YAML format
///       Example:
///       parameter:
///　　　　  software_demosaicing: "edge_sensing"
///
///       Configurable software_demosaicing values include default, no_color_processing,
///       nearest_neighbor、edge_sensing、hq_linear、rigorous、ipp、
///       directional_filter
std::optional<FlyCapture2::ColorProcessingAlgorithm> YamlPointGreyCameraSystemSetting::GetSoftDemosaicing() {
  const int32_t hash = __LINE__;
  if (cache_.count(hash)) {
    return std::any_cast<FlyCapture2::ColorProcessingAlgorithm>(cache_[hash]);
  }
  const YAML::Node& parameter = setting_node_["parameter"];
  if (!parameter || !parameter.IsMap()) {
    return std::nullopt;
  }
  const YAML::Node& software_demosaicing = parameter["software_demosaicing"];
  if (!software_demosaicing) {
    return std::nullopt;
  }

  std::unordered_map<std::string, FlyCapture2::ColorProcessingAlgorithm> color_processing_algorithms;
  color_processing_algorithms["default"] = FlyCapture2::DEFAULT;
  color_processing_algorithms["no_color_processing"] = FlyCapture2::NO_COLOR_PROCESSING;
  color_processing_algorithms["nearest_neighbor"] = FlyCapture2::NEAREST_NEIGHBOR;
  color_processing_algorithms["edge_sensing"] = FlyCapture2::EDGE_SENSING;
  color_processing_algorithms["hq_linear"] = FlyCapture2::HQ_LINEAR;
  color_processing_algorithms["rigorous"] = FlyCapture2::RIGOROUS;
  color_processing_algorithms["ipp"] = FlyCapture2::IPP;

  std::optional<FlyCapture2::ColorProcessingAlgorithm> software_demosaicing_opt;
  try {
    const std::string key = software_demosaicing.as<std::string>();
    if (color_processing_algorithms.count(key) == 0) {
      throw std::runtime_error((boost::format(
                                    "Invalid software demosaicing.\n"
                                    "software demosaicing: %1%") %
                                key).str());
    }
    software_demosaicing_opt = color_processing_algorithms[key];
    cache_[hash] = *software_demosaicing_opt;
  } catch (const YAML::Exception& e) {
    std::string description = "Failed to get software demosaicing.\nDetail: ";
    description = description + e.what();
    throw std::runtime_error(description);
  }

  return software_demosaicing_opt;
}

/// @brief Check if the software trigger is enabled
/// @return
/// Returns false if the software trigger is disabled or the setting does not exist
/// @exception std::runtime_error Thrown if the node is invalid
/// @note Information required for software trigger settings is in the following YAML format
///       Example:
///       parameter:
///　　　　  software_trigger: off
bool YamlPointGreyCameraSystemSetting::IsSoftwareTriggerEnabled() {
  const int32_t hash = __LINE__;
  if (cache_.count(hash)) {
    return std::any_cast<bool>(cache_[hash]);
  }
  const YAML::Node& parameter = setting_node_["parameter"];
  if (!parameter || !parameter.IsMap()) {
    return false;
  }
  const YAML::Node& software_trigger_node = parameter["software_trigger"];
  if (!software_trigger_node) {
    return false;
  }

  bool software_trigger = false;
  try {
    software_trigger = software_trigger_node.as<bool>();
  } catch (const YAML::Exception& e) {
    std::string description = "Failed to get software trigger setting.\nDetail: ";
    description = description + e.what();
    throw std::runtime_error(description);
  }

  cache_[hash] = software_trigger;

  return software_trigger;
}

/// @brief Check if the self-trigger is enabled
/// @return Returns false if the self-trigger is disabled or the setting does not exist
/// @exception std::runtime_error Thrown if the node is invalid
/// @note Information required for self-trigger settings is in the following YAML format
///       Example:
///       parameter:
///　　　　  self_trigger: on
bool YamlPointGreyCameraSystemSetting::IsSelfTriggerEnabled() {
  const int32_t hash = __LINE__;
  if (cache_.count(hash)) {
    return std::any_cast<bool>(cache_[hash]);
  }
  const YAML::Node& parameter = setting_node_["parameter"];
  if (!parameter || !parameter.IsMap()) {
    return false;
  }
  const YAML::Node& self_trigger_node = parameter["self_trigger"];
  if (!self_trigger_node) {
    return false;
  }

  bool self_trigger = false;
  try {
    self_trigger = self_trigger_node.as<bool>();
  } catch (const YAML::Exception& e) {
    std::string description = "Failed to get self trigger setting.\nDetail: ";
    description = description + e.what();
    throw std::runtime_error(description);
  }

  cache_[hash] = self_trigger;

  return self_trigger;
}

/// @brief Retrieve self-trigger settings
/// @return Retrieved self-trigger settings
///         Returns an invalid value if no setting exists
/// @exception std::runtime_error Thrown if the node key does not exist
/// @exception std::runtime_error Thrown if the node is invalid
/// @note Information required for self-trigger settings is in the following YAML format
///       Example:
///       self_trigger_io:
///         in_io: 0
///         out_io: 1
///         pulse_figure: 0x040000400
///         number_of_pulse: 0x01
///         polarity: 0
///
///       For Flea2, in_io=2 out_io=3
///       For Chameleon, Blackfly, in_io=0 out_io=1
std::optional<SelfTriggerSettings> YamlPointGreyCameraSystemSetting::GetSelfTriggerSettings() {
  const int32_t hash = __LINE__;
  if (cache_.count(hash)) {
    return std::any_cast<SelfTriggerSettings>(cache_[hash]);
  }
  const YAML::Node& parameter = setting_node_["parameter"];
  if (!parameter || !parameter.IsMap()) {
    return std::nullopt;
  }
  const YAML::Node& self_trigger_io = parameter["self_trigger_io"];
  if (!self_trigger_io) {
    return std::nullopt;
  }

  std::optional<SelfTriggerSettings> self_trigger_settings_opt = std::nullopt;
  try {
    SelfTriggerSettings self_trigger_settings = { 0, 0, 0, 0, 0 };
    const YAML::Node& in_io = self_trigger_io["in_io"];
    if (!in_io) {
      throw std::runtime_error("Not found key 'in_io' in self trigger setting.");
    }
    self_trigger_settings.in_io = in_io.as<uint32_t>();
    const YAML::Node& out_io = self_trigger_io["out_io"];
    if (!out_io) {
      throw std::runtime_error("Not found key 'out_io' in self trigger setting.");
    }
    self_trigger_settings.out_io = out_io.as<uint32_t>();
    const YAML::Node& pulse_figure = self_trigger_io["pulse_figure"];
    if (!pulse_figure) {
      throw std::runtime_error("Not found key 'pulse_figure' in self trigger setting.");
    }
    self_trigger_settings.pulse_figure = pulse_figure.as<uint32_t>();
    const YAML::Node& number_of_pulse = self_trigger_io["number_of_pulse"];
    if (!number_of_pulse) {
      throw std::runtime_error("Not found key 'number_of_pulse' in self trigger setting.");
    }
    self_trigger_settings.number_of_pulse = number_of_pulse.as<uint32_t>();

    const YAML::Node& polarity = self_trigger_io["polarity"];
    if (!polarity) {
      if (self_trigger_settings.number_of_pulse != 0x01) {
        throw std::runtime_error("Invalid setting 'self_trigger_io/number_of_pulse'");
      }
    } else {
      self_trigger_settings.polarity = polarity.as<uint32_t>();
    }
    if ((self_trigger_settings.polarity == 1 && self_trigger_settings.number_of_pulse != 0xFF) ||
        (self_trigger_settings.polarity == 0 && self_trigger_settings.number_of_pulse != 0x01)) {
      throw std::runtime_error("Invalid parameter combination. See self_trigger_io");
    }
    self_trigger_settings_opt = self_trigger_settings;
    cache_[hash] = *self_trigger_settings_opt;
  } catch (const YAML::Exception& e) {
    std::string description = "Failed to get self trigger settings.\nDetail: ";
    description = description + e.what();
    throw std::runtime_error(description);
  }

  return self_trigger_settings_opt;
}

/// @brief Retrieve trigger mode settings
/// @return Retrieved trigger mode settings
///         Returns an invalid value if no setting exists
/// @exception std::runtime_error Thrown if self-trigger usage is not properly configured
/// @exception std::runtime_error Thrown if the node is invalid
/// @note Information required for trigger mode settings is in the following YAML format
///       Example:
///       property:
///         trigger_mode:
///           onOff: on
///           mode: 0
///
///       Configurable values include onOff: bool, polarity: unsigned int,
///       source: unsigned int、mode: unsigned int、parameter: unsigned int、
///       reserved: int[8]
///
///       Additionally, if self-trigger is enabled, GetSelfTriggerSettings()
///       is called internally, so separate configuration is required
std::optional<FlyCapture2::TriggerMode> YamlPointGreyCameraSystemSetting::GetTriggerMode() {
  const int32_t hash = __LINE__;
  if (cache_.count(hash)) {
    return std::any_cast<FlyCapture2::TriggerMode>(cache_[hash]);
  }
  const YAML::Node& parameter = setting_node_["parameter"];
  if (!parameter || !parameter.IsMap()) {
    return std::nullopt;
  }
  const YAML::Node& trigger_mode = parameter["trigger_mode"];
  if (!trigger_mode) {
    return std::nullopt;
  }

  std::optional<FlyCapture2::TriggerMode> trigger_mode_opt;
  try {
    trigger_mode_opt = trigger_mode.as<FlyCapture2::TriggerMode>();

    if (trigger_mode_opt && trigger_mode_opt->onOff) {
      const std::optional<bool> is_software_trigger_enabled = IsSoftwareTriggerEnabled();
      const std::optional<bool> is_self_trigger_enabled = IsSelfTriggerEnabled();
      if (is_software_trigger_enabled && *is_software_trigger_enabled) {
        // Software trigger settings
        trigger_mode_opt->parameter = kSoftwareTriggerParameter;
        trigger_mode_opt->source = kSoftwareTriggerSource;
      } else if (is_self_trigger_enabled && *is_self_trigger_enabled) {
        const std::optional<SelfTriggerSettings> self_trigger_settings = GetSelfTriggerSettings();
        if (!self_trigger_settings) {
          throw std::runtime_error("Not found self trigger settings.");
        }
        // Retrieve the input IO number for self-trigger from the camera setting file
        trigger_mode_opt->parameter = 0;
        trigger_mode_opt->source = self_trigger_settings->in_io;
      } else {
        // Input IO number for external trigger settings is 0
        trigger_mode_opt->parameter = 0;
        trigger_mode_opt->source = 0;
      }
    }

    // If configured for Blackfly, ensure other parameters are also set for Blackfly
    const std::optional<SelfTriggerSettings> self_trigger_settings = GetSelfTriggerSettings();
    if (!self_trigger_settings) {
      throw std::runtime_error("Not found self trigger settings.");
    }
    if (((self_trigger_settings->polarity == 0 || self_trigger_settings->number_of_pulse == 0x01)
        && trigger_mode_opt->polarity == 1) ||
        ((self_trigger_settings->polarity == 1 || self_trigger_settings->number_of_pulse == 0xFF)
        && trigger_mode_opt->polarity == 0)) {
      throw std::runtime_error("Invalid parameter combination. See self_trigger_io and trigger_mode.");
    }
    cache_[hash] = *trigger_mode_opt;
  } catch (const YAML::Exception& e) {
    std::string description = "Failed to get trigger mode.\nDetail: ";
    description = description + e.what();
    throw std::runtime_error(description);
  }

  return trigger_mode_opt;
}

/// @brief Retrieve trigger delay settings
/// @return Retrieved trigger delay settings
///         Returns an invalid value if no setting exists
/// @exception std::runtime_error Thrown if the node is invalid
/// @note Information required for trigger delay settings is in the following YAML format
///       Example:
///       property:
///         trigger_mode:
///           absValue: 5.0
///           onOff: off
///
///       Configurable values include present: bool, absControl: bool, onePush: bool,
///       onOff: bool、autoManualMode: bool、valueA: unsigned int、
///       valueB: unsigned int、absValue: float、reserved: unsigned int[8]
std::optional<FlyCapture2::TriggerDelay> YamlPointGreyCameraSystemSetting::GetTriggerDelay() {
  const int32_t hash = __LINE__;
  if (cache_.count(hash)) {
    return std::any_cast<FlyCapture2::TriggerDelay>(cache_[hash]);
  }
  const YAML::Node& parameter = setting_node_["parameter"];
  if (!parameter || !parameter.IsMap()) {
    return std::nullopt;
  }
  const YAML::Node& trigger_delay = parameter["trigger_delay"];
  if (!trigger_delay) {
    return std::nullopt;
  }

  std::optional<FlyCapture2::TriggerDelay> trigger_delay_opt;
  try {
    trigger_delay_opt = trigger_delay.as<FlyCapture2::Property>();
    trigger_delay_opt->type = FlyCapture2::TRIGGER_DELAY;
    // absControl must be set to true, otherwise absValue cannot be configured
    trigger_delay_opt->absControl = true;

    cache_[hash] = *trigger_delay_opt;
  } catch (const YAML::Exception& e) {
    std::string description = "Failed to get trigger delay.\nDetail: ";
    description = description + e.what();
    throw std::runtime_error(description);
  }

  return trigger_delay_opt;
}

/// @brief Retrieve image type settings (monochrome, color)
/// @return Image type
///         Returns ImageType::kRgbImage if change_rgb_flag is on
///         Returns ImageType::kMonoImage if change_rgb_flag is off
///         Returns an invalid value if no setting exists
/// @exception std::runtime_error Thrown if the node is invalid
/// @note Information required for RGB conversion flag is in the following YAML format
///       Example:
///       property:
///         change_rgb_flag: on
std::optional<ImageType> YamlPointGreyCameraSystemSetting::GetImageType() {
  const int32_t hash = __LINE__;
  if (cache_.count(hash)) {
    return std::any_cast<ImageType>(cache_[hash]);
  }
  const YAML::Node& parameter = setting_node_["parameter"];
  if (!parameter || !parameter.IsMap()) {
    return std::nullopt;
  }
  const YAML::Node& change_rgb_flag_node = parameter["change_rgb_flag"];
  if (!change_rgb_flag_node) {
    return std::nullopt;
  }

  ImageType image_type = kMonoImage;
  try {
    if (change_rgb_flag_node.as<bool>()) {
      image_type = kRgbImage;
    }
  } catch (const YAML::Exception& e) {
    std::string description = "Failed to get image type.\nDetail: ";
    description = description + e.what();
    throw std::runtime_error(description);
  }

  cache_[hash] = image_type;

  return image_type;
}

/// @brief Retrieve 3.3V output settings
/// @return Whether to output or not
/// @note Output is possible only for Blackfly
std::optional<bool> YamlPointGreyCameraSystemSetting::GetOutputVoltageSetting() {
  const int32_t hash = __LINE__;
  if (cache_.count(hash)) {
    return std::any_cast<bool>(cache_[hash]);
  }
  const YAML::Node& parameter = setting_node_["parameter"];
  if (!parameter || !parameter.IsMap()) {
    return std::nullopt;
  }
  const YAML::Node& output_voltage_enable = parameter["output_voltage"];
  if (!output_voltage_enable) {
    return std::nullopt;
  }

  bool enable;
  try {
    enable = output_voltage_enable.as<bool>();
  } catch (const YAML::Exception& e) {
    std::string description = "Failed to get output voltage setting.\nDetail: ";
    description = description + e.what();
    throw std::runtime_error(description);
  }

  cache_[hash] = enable;

  return enable;
}

}  // end of namespace tmc_pgr_camera
