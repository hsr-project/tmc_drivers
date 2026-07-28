/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
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
/// @brief Issue camera diagnostics
/// @author Yuka Hashiguchi

#include <camera_diag_updater/camera_diag_component.hpp>

#include <tmc_utils/parameters.hpp>

namespace {
// Definition of default values for each variable
constexpr bool kUnicolorCheck = true;
constexpr double kDiagPubRate = 1.0;
constexpr double kDefaultBootTimeout = 60.0;
constexpr double kDefaultSubTimeout = 10.0;
constexpr uint32_t kMaxWindowSize = 10000;
constexpr double kDefaultFillingRate = 0.75;
constexpr uint8_t kDefaultCheckColorR = 0;
constexpr uint8_t kDefaultCheckColorG = 154;
constexpr uint8_t kDefaultCheckColorB = 0;
constexpr uint32_t kDefaultSamplingSizeWidth = 10;
constexpr uint32_t kDefaultSamplingSizeHeight = 10;
}  // unnamed namespace

namespace camera_diag_updater {

CameraDiagComponent::CameraDiagComponent(const rclcpp::NodeOptions& options)
    : Node("camera_diag_updater", options), updater_status_(kBooting),
      actual_hz_(0.0), unicolor_occurred_(false), launch_time_(this->now()),
      latest_sub_time_(rclcpp::Time(0, 0, RCL_ROS_TIME)) {
  onInit();
}

CameraDiagComponent::~CameraDiagComponent() {}

void CameraDiagComponent::onInit() {
  // Retrieve diagnostic parameters
  warning_hz_ = tmc_utils::GetParameter<double>(this, "warning_hz", 5.0);
  unicolor_check_ = tmc_utils::GetParameter<bool>(this, "unicolor_check", kUnicolorCheck);
  boot_timeout_ = tmc_utils::GetParameter<double>(this, "boot_timeout", kDefaultBootTimeout);
  sub_timeout_ = tmc_utils::GetParameter<double>(this, "sub_timeout", kDefaultSubTimeout);
  max_window_size_ = GetUInt32Param("max_window_size", kMaxWindowSize);
  filling_rate_ = tmc_utils::GetParameter<double>(this, "filling_rate", kDefaultFillingRate);
  color_check_r_ = GetUInt8Param("color_check_r", kDefaultCheckColorR);
  color_check_g_ = GetUInt8Param("color_check_g", kDefaultCheckColorG);
  color_check_b_ = GetUInt8Param("color_check_b", kDefaultCheckColorB);
  sampling_size_x_ = GetUInt32Param("sampling_size_x", kDefaultSamplingSizeWidth);
  sampling_size_y_ = GetUInt32Param("sampling_size_y", kDefaultSamplingSizeHeight);

  // Initialize subscriber
  const auto topic_name = tmc_utils::GetParameter<std::string>(this, "topic_name", "image");
  const auto topic_type = tmc_utils::GetParameter<std::string>(this, "topic_type", "image");
  const auto hardware_id = tmc_utils::GetParameter<std::string>(this, "hardware_id", topic_name);

  if (topic_type == "image") {
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_name, 10, std::bind(&CameraDiagComponent::ImageCallback, this, std::placeholders::_1));
  } else if (topic_type == "points") {
    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name, 10, std::bind(&CameraDiagComponent::PointsCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(this->get_logger(), "[%s]Invalid topic type:%s", hardware_id.c_str(), topic_type.c_str());
    return;
  }

  // Start diagnostic updater
  const auto diag_pub_rate = tmc_utils::GetParameter<double>(this, "diag_pub_rate", kDiagPubRate);

  updater_.reset(new diagnostic_updater::Updater(this));
  updater_->setHardwareID(hardware_id);
  updater_->add(hardware_id + " topic status", this, &CameraDiagComponent::ProduceDiagnostics);
  diag_updater_timer_ = this->create_wall_timer(
    rclcpp::Duration::from_seconds(1.0 / diag_pub_rate).to_chrono<std::chrono::nanoseconds>(),
    std::bind(&CameraDiagComponent::Run, this));
}

uint8_t CameraDiagComponent::GetUInt8Param(const std::string& name, uint8_t default_value) {
  const auto param_data = tmc_utils::GetParameter<int32_t>(this, name, default_value);
  if (param_data >= 0) {
    return static_cast<uint8_t>(param_data);
  } else {
    RCLCPP_WARN(this->get_logger(), "Parameter %s should not be negative value. Use default setting.", name.c_str());
    return default_value;
  }
}

uint32_t CameraDiagComponent::GetUInt32Param(const std::string& name, uint32_t default_value) {
  const auto param_data = tmc_utils::GetParameter<int32_t>(this, name, default_value);
  if (param_data >= 0) {
    return static_cast<uint32_t>(param_data);
  } else {
    RCLCPP_WARN(this->get_logger(), "Parameter %s should not be negative value. Use default setting.", name.c_str());
    return default_value;
  }
}

void CameraDiagComponent::Run() {
  CalculateHz();
  updater_->force_update();
}

void CameraDiagComponent::ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& dst_stat) {
  std::scoped_lock<std::mutex> lock(mutex_);
  switch (updater_status_) {
    case kBooting: {
      dst_stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Booting");
      break;
    }
    case kBootError: {
      dst_stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Boot timeout");
      break;
    }
    case kUpdating: {
      if (unicolor_occurred_) {
        dst_stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Fill image with unicolor");
      } else {
        // Branching by frequency
        if (std::isnan(actual_hz_)) {
          dst_stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                           "Only one message is subscribed. Cannot calculate frequency.");
        } else if (actual_hz_ <= std::numeric_limits<double>::min()) {
          dst_stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No New Messages");
        } else if (actual_hz_ < warning_hz_) {
          dst_stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Subscribing Rate is slow");
        } else {
          dst_stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
        }
      }
      break;
    }
  }

  auto latest_sub_time_seconds = RCL_NS_TO_S(latest_sub_time_.nanoseconds());
  auto latest_sub_time_nanoseconds = latest_sub_time_.nanoseconds() - RCL_S_TO_NS(latest_sub_time_seconds);

  dst_stat.add("Latest Subscribed Time [sec]", latest_sub_time_seconds);
  dst_stat.add("Latest Subscribed Time [nanosec]", latest_sub_time_nanoseconds);
  dst_stat.add("Actual frequency (Hz)", actual_hz_);
  dst_stat.add("Events in window", times_.size());
  dst_stat.add("Minimum acceptable frequency (Hz)", warning_hz_);
}

void CameraDiagComponent::CalculateHz() {
  std::scoped_lock<std::mutex> lock(mutex_);
  switch (updater_status_) {
    case kBooting: {
      // At startup
      if ((this->now() - launch_time_).seconds() > boot_timeout_) {
        actual_hz_ = std::numeric_limits<double>::quiet_NaN();
        updater_status_ = kBootError;
      }
      break;
    }
    case kBootError: {
      // No updates
      return;
    }
    case kUpdating: {
      if ((this->now()  - latest_sub_time_).seconds() > sub_timeout_) {
        unicolor_occurred_ = false;
        actual_hz_ = 0.0;
        times_.clear();
        return;
      }
      uint32_t size = times_.size();
      if (size == 1) {
        // Insert NaN and issue a warning as calculation is not possible
        actual_hz_ = std::numeric_limits<double>::quiet_NaN();
      } else if (times_.back() > times_.front()) {
        actual_hz_ = (size - 1) / (times_.back() - times_.front());
      }
    }
  }
}

void CameraDiagComponent::UpdateLatestSubTime(const rclcpp::Time& current_time) {
  std::scoped_lock<std::mutex> lock(mutex_);
  latest_sub_time_ = current_time;
  times_.push_back(current_time.seconds());
  // Remove old data if the number of data exceeds the defined maximum value
  if (times_.size() > max_window_size_) times_.pop_front();
}

void CameraDiagComponent::CheckUnicolor(const sensor_msgs::msg::Image& img) {
  uint8_t color1;
  uint8_t color2;
  uint8_t color3;

  // TODO(yuka_hashiguchi) エンコーディングの形式に対する拡張性向上
  if (img.encoding == "rgb8") {
    color1 = color_check_r_;
    color2 = color_check_g_;
    color3 = color_check_b_;
  } else if (img.encoding == "bgr8") {
    color1 = color_check_b_;
    color2 = color_check_g_;
    color3 = color_check_r_;
  } else {
    RCLCPP_WARN_ONCE(this->get_logger(), "Unicolor check only support rgb8 or bgr8 encoding. Stop checking.");
    return;
  }

  uint32_t unicolor_pixel = 0;
  uint8_t channel = img.step / img.width;
  // Check the RGB value of the top-left pixel of the sampled block
  for (uint32_t y = 0; y < img.height; y += sampling_size_y_) {
    for (uint32_t x = 0; x < img.width; x += sampling_size_x_) {
      uint32_t index = channel * (x + img.width * y);
      // TODO(yuka_hashiguchi) エンコーディングのチャンネル数に対する拡張性向上
      if (img.data[index] == color1 && img.data[index + 1] == color2 && img.data[index + 2] == color3) {
        unicolor_pixel++;
      }
    }
  }
  if (unicolor_pixel / ((img.height / sampling_size_y_) * (img.width / sampling_size_x_)) > filling_rate_) {
    unicolor_occurred_ = true;
  } else {
    unicolor_occurred_ = false;
  }
}

void CameraDiagComponent::ImageCallback(const sensor_msgs::msg::Image& img) {
  updater_status_ = kUpdating;
  UpdateLatestSubTime(this->now());
  if (unicolor_check_) CheckUnicolor(img);
}

void CameraDiagComponent::PointsCallback(const sensor_msgs::msg::PointCloud2& img) {
  updater_status_ = kUpdating;
  UpdateLatestSubTime(this->now());
}
}  // namespace camera_diag_updater

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_diag_updater::CameraDiagComponent)
