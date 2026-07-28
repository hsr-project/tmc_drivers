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
/// @brief Class for issuing camera diagnostics
/// @author Yuka Hashiguchi

#ifndef CAMERA_DIAG_UPDATER_CAMERA_DIAG_COMPONENT_HPP__
#define CAMERA_DIAG_UPDATER_CAMERA_DIAG_COMPONENT_HPP__

#include <deque>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace camera_diag_updater {
class CameraDiagComponent : public rclcpp::Node {
 public:
  /// Constructor
  explicit CameraDiagComponent(const rclcpp::NodeOptions& options);
  /// Destructor
  virtual ~CameraDiagComponent();

 private:
  /// Initialization process
  virtual void onInit();

  /// Retrieve a parameter of type uint8_t
  /// @param [in] name Parameter name
  /// @param [in] default_value Default value of the parameter
  uint8_t GetUInt8Param(const std::string& name, uint8_t default_value);

  /// Retrieve a parameter of type uint32_t
  /// @param [in] name Parameter name
  /// @param [in] default_value Default value of the parameter
  uint32_t GetUInt32Param(const std::string& name, uint32_t default_value);

  /// Update diagnostics with a timer
  void Run();

  /// Generate diagnostics
  /// @param [out] dst_stat Status of the diagnostics to be issued
  void ProduceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& dst_stat);

  /// Calculate the topic subscription period
  void CalculateHz();

  /// Update the last subscription time
  /// @param [in] current_time Time when the topic callback function was entered
  void UpdateLatestSubTime(const rclcpp::Time& current_time);

  /// Check if the image is filled with a single color
  /// @param [in] img Message data
  void CheckUnicolor(const sensor_msgs::msg::Image& img);

  /// Callback function of type sensor_msgs::Image
  /// @param [in] img Message data
  void ImageCallback(const sensor_msgs::msg::Image& img);

  /// Callback function of type sensor_msgs::PointCloud2
  /// @param [in] points Message data
  void PointsCallback(const sensor_msgs::msg::PointCloud2& points);


  /// State of the diagnostic updater
  enum UpdaterStatus {
    kBooting,
    kBootError,
    kUpdating
  };

  /// Nodelet startup time
  const rclcpp::Time launch_time_;
  /// Diagnostic updater
  std::shared_ptr<diagnostic_updater::Updater> updater_;
  /// Variable to hold the diagnostic state (UpdaterStatus)
  uint8_t updater_status_;
  // Periodic event issuance for Diag update
  rclcpp::TimerBase::SharedPtr diag_updater_timer_;
  /// Subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
  /// Definition of topic subscription period WARN level [Hz]
  double warning_hz_;
  /// Maximum number of topics used to calculate the period
  uint32_t max_window_size_;
  /// Last subscription time
  rclcpp::Time latest_sub_time_;
  /// Mutex
  std::mutex mutex_;
  /// Queue to store the time when the (index + 1)th message was received
  std::deque<double> times_;
  /// Measured value of topic subscription period [Hz]
  double actual_hz_;
  /// Allowable time to subscribe to the first message [s]
  double boot_timeout_;
  /// Value to determine the topic has stopped if no message is subscribed within this time [s]
  double sub_timeout_;

  /// Variable for detecting the phenomenon of an image being filled with a single color
  /// Whether to check or not
  bool unicolor_check_;
  /// Detect the phenomenon if filled with a single color above this ratio
  double filling_rate_;
  /// Target R value
  uint8_t color_check_r_;
  /// Target G value
  uint8_t color_check_g_;
  /// Target B value
  uint8_t color_check_b_;
  /// Sampling size in the width direction
  uint32_t sampling_size_x_;
  /// Sampling size in the height direction
  uint32_t sampling_size_y_;
  /// Flag indicating whether the phenomenon is occurring
  bool unicolor_occurred_;
};
}  // namespace camera_diag_updater

#endif  // CAMERA_DIAG_UPDATER_CAMERA_DIAG_COMPONENT_HPP__
