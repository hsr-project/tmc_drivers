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
#include "tmc_pgr_camera/ros_parameter_point_grey_camera_system_setting.hpp"
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <boost/format.hpp>
#include <wordexp.h>

namespace tmc_pgr_camera {

/// @brief Constructor
/// @param[in] node_handle ROS node handle
/// @exception std::invalid_argument If there is a contradiction in the configuration
RosParameterPointGreyCameraSystemSetting::RosParameterPointGreyCameraSystemSetting(rclcpp::Node::SharedPtr node_handle)
    : node_handle_(node_handle) {
}

/// @brief Obtain an array of serial numbers for cameras used in the camera system
/// @return Array of camera serial numbers
/// @note The allowable number of elements is only 1 (monocular) or 2 (stereo)
/// @note In the case of 2 elements, the first element becomes the master
/// @exception std::runtime_error If the array elements are not acceptable
std::vector<uint32_t> RosParameterPointGreyCameraSystemSetting::GetSerialNumbers() {
  if (!serials_.empty()) {
    return serials_;
  }
  std::vector<int64_t> buffer;
  if (node_handle_->has_parameter("camera")) {
      node_handle_->get_parameter("camera", buffer);
  }
  if (buffer.size() == 0 || buffer.size() > 2) {
    throw std::runtime_error("Invalid serial setting length.");
  }

  for (std::vector<int64_t>::iterator it = buffer.begin(); it != buffer.end(); ++it) {
    if (*it < 0) {
      throw std::runtime_error("Minus id is invalid");
    }
  }

  for (std::vector<int64_t>::iterator it = buffer.begin(); it != buffer.end(); ++it) {
    serials_.push_back(static_cast<uint32_t>(*it));
  }

  return serials_;
}

/// @brief Load an array of camera properties
/// @return Array of camera properties
std::vector<FlyCapture2::Property> RosParameterPointGreyCameraSystemSetting::GetProperties() {
  if (!properties_.empty()) {
    return properties_;
  }
  std::vector<std::pair<std::string, FlyCapture2::PropertyType> > types;
  types.push_back(std::make_pair("property.brightness", FlyCapture2::BRIGHTNESS));
  types.push_back(std::make_pair("property.auto_exposure", FlyCapture2::AUTO_EXPOSURE));
  types.push_back(std::make_pair("property.sharpness", FlyCapture2::SHARPNESS));
  types.push_back(std::make_pair("property.white_balance", FlyCapture2::WHITE_BALANCE));
  types.push_back(std::make_pair("property.hue", FlyCapture2::HUE));
  types.push_back(std::make_pair("property.saturation", FlyCapture2::SATURATION));
  types.push_back(std::make_pair("property.gamma", FlyCapture2::GAMMA));
  types.push_back(std::make_pair("property.iris", FlyCapture2::IRIS));
  types.push_back(std::make_pair("property.focus", FlyCapture2::FOCUS));
  types.push_back(std::make_pair("property.zoom", FlyCapture2::ZOOM));
  types.push_back(std::make_pair("property.pan", FlyCapture2::PAN));
  types.push_back(std::make_pair("property.tilt", FlyCapture2::TILT));
  types.push_back(std::make_pair("property.shutter", FlyCapture2::SHUTTER));
  types.push_back(std::make_pair("property.gain", FlyCapture2::GAIN));
  types.push_back(std::make_pair("property.trigger_mode", FlyCapture2::TRIGGER_MODE));
  types.push_back(std::make_pair("property.trigger_delay", FlyCapture2::TRIGGER_DELAY));
  types.push_back(std::make_pair("property.frame_rate", FlyCapture2::FRAME_RATE));
  types.push_back(std::make_pair("property.temperature", FlyCapture2::TEMPERATURE));

  std::vector<FlyCapture2::Property> camera_properties;
  for (std::vector<std::pair<std::string, FlyCapture2::PropertyType> >::iterator it = types.begin();
      it != types.end(); ++it) {
    FlyCapture2::Property property;
    if (node_handle_->has_parameter(it->first + std::string(".on_off"))) {
      property.onOff =
        node_handle_->get_parameter(it->first + std::string(".on_off")).as_bool();
      if (node_handle_->has_parameter(it->first + std::string(".abs_control"))) {
        property.absControl =
            node_handle_->get_parameter(it->first + std::string(".abs_control")).as_bool();
      }
      if (node_handle_->has_parameter(it->first + std::string(".abs_value"))) {
        property.absValue =
            node_handle_->get_parameter(it->first + std::string(".abs_value")).as_double();
      }
      if (node_handle_->has_parameter(it->first + std::string(".one_push"))) {
        property.onePush =
            node_handle_->get_parameter(it->first + std::string(".one_push")).get_value<bool>();
      }
      if (node_handle_->has_parameter(it->first + std::string(".present"))) {
        property.present =
            node_handle_->get_parameter(it->first + std::string(".present")).get_value<bool>();
      }
      property.type = it->second;
      if (node_handle_->has_parameter(it->first + std::string(".auto_manual_mode"))) {
        property.autoManualMode =
            node_handle_->get_parameter(it->first + std::string(".auto_manual_mode")).get_value<bool>();
      }
      if (node_handle_->has_parameter(it->first + std::string(".value_a")) &&
            node_handle_->has_parameter(it->first + std::string(".value_b"))) {
        property.valueA =
            node_handle_->get_parameter(it->first + std::string(".value_a")).as_int();
        property.valueB =
            node_handle_->get_parameter(it->first + std::string(".value_b")).as_int();
      }

      camera_properties.push_back(property);
    }
  }

  if (static_cast<int>(camera_properties.size()) == 0) {
    throw std::runtime_error("No properties in parameter server.");
  }

  properties_ = camera_properties;

  return camera_properties;
}

/// @brief Obtain the frame rate
/// @param[in] video_mode The video format actually used
/// @return A pair of the constant value in the FlyCapture2 SDK for the obtained frame rate
///         and the actual frame rate value. In the case of Format7, the constant value is FRAMERATE_FORMAT7
/// @exception std::runtime_error If the parameter does not exist
/// @exception std::runtime_error If a non-existent frame rate is set (only judged when something other than FORMAT7 is set)
std::optional<std::pair<FlyCapture2::FrameRate, float> > RosParameterPointGreyCameraSystemSetting::GetFrameRate(
    const FlyCapture2::VideoMode video_mode) {
  if (frame_rate_) {
    return frame_rate_;
  }
  float frame_rate;
  if (node_handle_->has_parameter("frame_rate")) {
    frame_rate = node_handle_->get_parameter("frame_rate").get_value<double>();
  } else {
    throw std::runtime_error("In parameter server, there is no frame_rate parameter.");
  }
  if (frame_rate < std::numeric_limits<float>::epsilon()) {
    throw std::runtime_error("Invalid frame_rate.");
  }
  if (video_mode == FlyCapture2::VIDEOMODE_FORMAT7) {
    frame_rate_ =  std::make_pair(FlyCapture2::FRAMERATE_FORMAT7, frame_rate);
    return frame_rate_;
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
  if (frame_rates.count(frame_rate) == 0) {
    throw std::runtime_error((boost::format("Invalid frame rate.\nvalue: %1%") % frame_rate).str());
  }
  frame_rate_ =  std::make_pair(frame_rates[frame_rate], frame_rate);

  return frame_rate_;
}

/// @brief Obtain the video mode
/// @return The constant value in the FlyCapture2 SDK for the obtained video mode
///         Returns an invalid value if there is no setting
/// @exception std::runtime_error If the parameter does not exist
/// @exception std::runtime_error If a non-existent video mode is specified
std::optional<FlyCapture2::VideoMode> RosParameterPointGreyCameraSystemSetting::GetVideoMode() {
  if (video_mode_) {
    return video_mode_;
  }
  std::string video_mode;
  if (node_handle_->has_parameter("video_mode")) {
    video_mode = node_handle_->get_parameter("video_mode").get_value<std::string>();
  } else {
    throw std::runtime_error("In parameter server, there is no video_mode parameter.");
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
  if (video_modes.count(video_mode) == 0) {
    throw std::runtime_error((boost::format("Invalid video mode.\nmode: %1%") % video_mode).str());
  }
  video_mode_ = video_modes[video_mode];

  return video_mode_;
}

/// @brief Obtain the Format7 settings
/// @return Obtained Format7 settings
///         Returns an invalid value if the setting does not exist
/// @exception std::runtime_error If the parameter does not exist
/// @exception std::runtime_error If the node key does not exist
std::optional<FlyCapture2::Format7ImageSettings> RosParameterPointGreyCameraSystemSetting::GetFormat7Setting() {
  if (format7_setting_) {
    return format7_setting_;
  }
  std::vector<int> format7_without_pixel_format(5);
  std::vector<std::string> properties;
  properties.push_back("mode");
  properties.push_back("offset_x");
  properties.push_back("offset_y");
  properties.push_back("width");
  properties.push_back("height");
  std::vector<int>::iterator it_format7_without_pixel_format = format7_without_pixel_format.begin();
  std::vector<std::string>::iterator it_properties = properties.begin();
  for (; it_properties != properties.end(); ++it_format7_without_pixel_format, ++it_properties) {
    if (node_handle_->has_parameter(std::string("format7.") + *it_properties)) {
      *it_format7_without_pixel_format = static_cast<int>(
          node_handle_->get_parameter(std::string("format7.") + *it_properties).as_int());
    } else {
      throw std::runtime_error(
          std::string("In parameter server, there is no format7.") + *it_properties + std::string("parameter."));
    }
  }
  if (format7_without_pixel_format.at(0) < 0 || 31 < format7_without_pixel_format.at(0)) {
      throw std::runtime_error(
          "Invalid mode of format7 in configuration YAML file.\n"
          "Set mode from 0 to 31.");
  }
  FlyCapture2::Format7ImageSettings format7;
  format7.mode = static_cast<FlyCapture2::Mode>(format7_without_pixel_format.at(0));
  format7.offsetX = format7_without_pixel_format.at(1);
  format7.offsetY = format7_without_pixel_format.at(2);
  format7.width = format7_without_pixel_format.at(3);
  format7.height = format7_without_pixel_format.at(4);

  std::string pixel_format;
  if (node_handle_->has_parameter("format7.pixel_format")) {
    pixel_format = node_handle_->get_parameter("format7.pixel_format").get_value<std::string>();
  } else {
      throw std::runtime_error("In parameter server, there is no format7.pixel_format parameter.");
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
  if (pixel_formats.count(pixel_format) == 0) {
    throw std::runtime_error((boost::format("Unknown pixel format '%1%'.") % pixel_format).str());
  }
  format7.pixelFormat = pixel_formats[pixel_format];
  format7_setting_ = format7;

  return format7_setting_;
}

/// @brief Obtain the software demosaicing settings
/// @return Obtained software demosaicing settings
///         Returns an invalid value if the setting does not exist
/// @exception std::runtime_error If the parameter does not exist
/// @exception std::runtime_error If an incorrect string is specified for the setting value
std::optional<FlyCapture2::ColorProcessingAlgorithm> RosParameterPointGreyCameraSystemSetting::GetSoftDemosaicing() {
  if (software_demosaicing_) {
    return software_demosaicing_;
  }
  std::string software_demosaicing;
  if (node_handle_->has_parameter("software_demosaicing")) {
    software_demosaicing = node_handle_->get_parameter("software_demosaicing").get_value<std::string>();
  } else {
    throw std::runtime_error("In parameter server, there is no software_demosaicing parameter.");
  }
  std::unordered_map<std::string, FlyCapture2::ColorProcessingAlgorithm> color_processing_algorithms;
  color_processing_algorithms["default"] = FlyCapture2::DEFAULT;
  color_processing_algorithms["no_color_processing"] = FlyCapture2::NO_COLOR_PROCESSING;
  color_processing_algorithms["nearest_neighbor"] = FlyCapture2::NEAREST_NEIGHBOR;
  color_processing_algorithms["edge_sensing"] = FlyCapture2::EDGE_SENSING;
  color_processing_algorithms["hq_linear"] = FlyCapture2::HQ_LINEAR;
  color_processing_algorithms["rigorous"] = FlyCapture2::RIGOROUS;
  color_processing_algorithms["ipp"] = FlyCapture2::IPP;
  if (color_processing_algorithms.count(software_demosaicing) == 0) {
    throw std::runtime_error((boost::format(
                                  "Invalid software demosaicing.\n"
                                  "software demosaicing: %1%") %
                              software_demosaicing).str());
  }
  software_demosaicing_ = color_processing_algorithms[software_demosaicing];
  return software_demosaicing_;
}

/// @brief Check if the software trigger is enabled
/// @return Returns false if the software trigger is disabled or the setting does not exist
/// @note Depends on the number of elements in the camera serial array (enabled if 1)
bool RosParameterPointGreyCameraSystemSetting::IsSoftwareTriggerEnabled() {
  std::vector<uint32_t> camera_serial = GetSerialNumbers();
  return static_cast<int>(camera_serial.size()) == 1;
}

/// @brief Check if the self-trigger is enabled
/// @return Returns false if the self-trigger is disabled or the setting does not exist
/// @note Depends on the number of elements in the camera serial array (enabled if 2)
bool RosParameterPointGreyCameraSystemSetting::IsSelfTriggerEnabled() {
  std::vector<uint32_t> camera_serial = GetSerialNumbers();
  return static_cast<int>(camera_serial.size()) == 2;
}

/// @brief Obtain the self-trigger settings
/// @return Obtained self-trigger settings
///         Returns an invalid value if the setting does not exist
/// @exception std::runtime_error If the parameter is invalid
std::optional<SelfTriggerSettings> RosParameterPointGreyCameraSystemSetting::GetSelfTriggerSettings() {
  if (self_trigger_setting_) {
    return self_trigger_setting_;
  }
  // Input/Output pin number.
  std::vector<int> io;
  if (node_handle_->has_parameter("self_trigger.io")) {
    std::vector<int64_t> io_param;
    node_handle_->get_parameter("self_trigger.io", io_param);
    for (size_t i = 0; i < io_param.size(); i++) {
      io.push_back(static_cast<int>(io_param[i]));
    }
  }
  if (static_cast<int>(io.size()) != 2) {
    throw std::runtime_error("Invalid self_trigger io vector length.");
  }
  if (io.at(0) < 0 || io.at(1) < 0) {
    throw std::runtime_error("Invalid self_trigger io value.");
  }
  SelfTriggerSettings self_trigger_setting;
  self_trigger_setting.in_io = io.at(0);
  self_trigger_setting.out_io = io.at(1);

  // Pulse width (high, low)
  std::vector<int> width;
  if (node_handle_->has_parameter("self_trigger.pulse_width")) {
    std::vector<int64_t> width_param;
    node_handle_->get_parameter("self_trigger.pulse_width", width_param);
    for (size_t i = 0; i < width_param.size(); i++) {
      width.push_back(static_cast<int>(width_param[i]));
    }
  }
  if (static_cast<int>(width.size()) != 2) {
    throw std::runtime_error("Invalid self_trigger pulse width vector length.");
  }
  // Oscillation period setting
  // Format is 0xLLLLHHHH (L affects the time it is Low, H affects the time it is High)
  // Unit is 9.765e-7[sec] 1.024e+6[Hz]
  // The individual setting range for Low and High is
  // min: 9.765e-7[sec] 1.024e+6[Hz]
  // max: 0.063999[sec] 15.625[Hz] (min x 0xFFFF)
  // In other words, as a whole waveform
  // min: 1.9531e-6[sec] 5.12e+5[Hz]
  // max: 0.1279980[sec] 7.8126[Hz]
  // As a rosparam, it can be set in msec units
  // (To simplify processing, as there is no need for such detailed adjustments)
  for (std::vector<int>::iterator it = width.begin(); it != width.end(); ++it) {
    if (*it < 1 || 63 < *it) {  // The allowable range is 1msec or more and 63msec or less
      throw std::runtime_error("Invalid self_trigger io value.");
    }
  }
  // Since Low and High need to be written to the register together, they are integrated
  // 1[msec] = 0x0400(1024)
  uint32_t high = width.at(0) * 1024;
  uint32_t low = width.at(1) * 1024;
  self_trigger_setting.pulse_figure = (0xFFFF0000 & low << 16) | (0x0000FFFF & high);

  // Number of pulse
  int number_of_pulse = 0;
  if (node_handle_->has_parameter("self_trigger.number_of_pulse")) {
    number_of_pulse = static_cast<int>(node_handle_->get_parameter("self_trigger.number_of_pulse").as_int());
  }
  self_trigger_setting.number_of_pulse = number_of_pulse;
  if (self_trigger_setting.number_of_pulse <= 0x00 || 0xFF < self_trigger_setting.number_of_pulse) {
    throw std::runtime_error("Invalid self_trigger number_of_pulse.");
  }

  // Polarity
  int polarity = 0;
  if (node_handle_->has_parameter("self_trigger.polarity")) {
    polarity = static_cast<int>(node_handle_->get_parameter("self_trigger.polarity").as_int());
  }
  if (polarity < 0 || 1 < polarity) {
    throw std::runtime_error("Invalid self_trigger polarity.");
  }
  self_trigger_setting.polarity = polarity;

  self_trigger_setting_ = self_trigger_setting;
  return self_trigger_setting_;
}

/// @brief Obtain the trigger mode settings
/// @return Obtained trigger mode settings
/// @exception std::runtime_error If the parameter is invalid (mode)
/// @exception std::runtime_error If the parameter does not exist
std::optional<FlyCapture2::TriggerMode> RosParameterPointGreyCameraSystemSetting::GetTriggerMode() {
  std::optional<FlyCapture2::TriggerMode> trigger_mode;
  {
    std::shared_lock<std::shared_mutex> read(access_);
    trigger_mode = trigger_mode_;
  }
  if (trigger_mode) {
    return trigger_mode;
  }

  int trigger_mode_mode;
  if (node_handle_->has_parameter("trigger_mode.mode")) {
    trigger_mode_mode = static_cast<int>(node_handle_->get_parameter("trigger_mode.mode").as_int());
  } else {
    throw std::runtime_error("In parameter server, there is no trigger_mode.mode parameter.");
  }
  bool trigger_mode_on_off;
  if (node_handle_->has_parameter("trigger_mode.on_off")) {
    trigger_mode_on_off = node_handle_->get_parameter("trigger_mode.on_off").as_bool();
  } else {
      throw std::runtime_error("In parameter server, there is no trigger_mode.on_off parameter.");
  }
  int trigger_mode_polarity;
  if (node_handle_->has_parameter("trigger_mode.polarity")) {
    trigger_mode_polarity = static_cast<int>(node_handle_->get_parameter("trigger_mode.polarity").as_int());
  } else {
    throw std::runtime_error("In parameter server, there is no trigger_mode.polarity parameter.");
  }

  UpdateTriggerMode(trigger_mode_mode, trigger_mode_on_off, trigger_mode_polarity);

  {
    std::shared_lock<std::shared_mutex> read(access_);
    trigger_mode = trigger_mode_;
  }
  return trigger_mode;
}

/// @brief Update the trigger mode settings
/// @exception std::runtime_error If the parameter is invalid (mode)
/// @exception std::runtime_error If the parameter does not exist
void RosParameterPointGreyCameraSystemSetting::UpdateTriggerMode(int trigger_mode_mode,
                                                                 bool trigger_mode_on_off,
                                                                 int trigger_mode_polarity) {
  FlyCapture2::TriggerMode trigger_mode;
  if (trigger_mode_mode < 0 || 14 < trigger_mode_mode) {
    throw std::runtime_error("Invalid trigger_mode.mode");
  }
  if (trigger_mode_polarity < 0 || 1 < trigger_mode_polarity) {
    throw std::runtime_error("Invalid trigger_mode.polarity");
  }

  trigger_mode.mode = trigger_mode_mode;
  trigger_mode.onOff = trigger_mode_on_off;
  trigger_mode.polarity = trigger_mode_polarity;

  if (trigger_mode_on_off) {
    const std::optional<bool> is_software_trigger_enabled = IsSoftwareTriggerEnabled();
    const std::optional<bool> is_self_trigger_enabled = IsSelfTriggerEnabled();
    if (is_software_trigger_enabled && *is_software_trigger_enabled) {
      trigger_mode.parameter = 1;
      trigger_mode.source = 7;
    } else if (is_self_trigger_enabled && *is_self_trigger_enabled) {
      const std::optional<SelfTriggerSettings> self_trigger_settings = GetSelfTriggerSettings();
      if (!self_trigger_settings) {
        throw std::runtime_error("Not found self trigger settings.");
      }
      trigger_mode.parameter = 0;
      trigger_mode.source = self_trigger_settings->in_io;
    } else {
      trigger_mode.parameter = 0;
      trigger_mode.source = 0;
    }
  }
  {
    std::unique_lock<std::shared_mutex> write(access_);
    trigger_mode_ = trigger_mode;
  }
}

/// @brief Obtain the trigger delay settings
/// @return Obtained trigger delay settings
///         Returns an invalid value if the setting does not exist
std::optional<FlyCapture2::TriggerDelay> RosParameterPointGreyCameraSystemSetting::GetTriggerDelay() {
  std::vector<FlyCapture2::Property> properties = GetProperties();
  for (std::vector<FlyCapture2::Property>::iterator it = properties.begin(); it != properties.end(); ++it) {
    if (it->type == FlyCapture2::TRIGGER_DELAY) {
      return std::optional<FlyCapture2::TriggerDelay>(*it);
    }
  }
  throw std::runtime_error("Could not get trigger delay.");
}

/// @brief Obtain the image type settings (monochrome, color)
/// @return Image type
///         Returns ImageType::kRgbImage if change_rgb_flag is on
///         Returns ImageType::kMonoImage if off
///         Returns an invalid value if the setting does not exist
/// @exception std::runtime_error If the parameter does not exist
std::optional<ImageType> RosParameterPointGreyCameraSystemSetting::GetImageType() {
  if (image_type_) {
    return image_type_;
  }
  bool change_rgb_flag;
  if (node_handle_->has_parameter("change_rgb_flag")) {
    change_rgb_flag = node_handle_->get_parameter("change_rgb_flag").as_bool();
  } else {
      throw std::runtime_error("In parameter server, there is no change_rgb_flag parameter.");
  }
  ImageType image_type = kMonoImage;
  if (change_rgb_flag) {
    image_type = kRgbImage;
  }
  image_type_ = image_type;

  return image_type;
}

/// @brief Obtain the 3.3V output settings
/// @return Whether to output or not
/// @note Output is possible only for Blackfly
std::optional<bool> RosParameterPointGreyCameraSystemSetting::GetOutputVoltageSetting() {
  if (output_voltage_enable_) {
    return output_voltage_enable_;
  }

  bool output_voltage_enable;
  if (node_handle_->has_parameter("output_voltage")) {
    output_voltage_enable = node_handle_->get_parameter("output_voltage").as_bool();
  } else {
      throw std::runtime_error("In parameter server, there is no output_voltage parameter.");
  }

  output_voltage_enable_ = output_voltage_enable;

  return output_voltage_enable_;
}
}  // end of namespace tmc_pgr_camera
