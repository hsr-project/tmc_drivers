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
/// @brief Test fixture for camera diagnostics

#ifndef CAMERA_DIAG_UPDATER_TEST_FIXTURE_HPP_
#define CAMERA_DIAG_UPDATER_TEST_FIXTURE_HPP_

#include <chrono>
#include <string>

#include <boost/circular_buffer.hpp>
#include <gtest/gtest.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/// Topic name for diagnostics
extern const char* const kDiagTopicName = "/diagnostics";
/// Dummy camera topic name (image)
extern const char* const kCameraImageTopicName = "camera/rgb/image_rect_color";
/// Dummy camera topic name (points)
extern const char* const kCameraPointsTopicName = "camera/depth_registered/points";
/// Dummy camera hardware ID (image)
extern const char* const kCameraImageHardwareID = "rgb/image_rect_color";
/// Dummy camera hardware ID (points)
extern const char* const kCameraPointsHardwareID = "depth_registered/points";
/// Dummy camera size (height)
extern const uint8_t kCameraHeightSize = 10;
/// Dummy camera size (width)
extern const uint8_t kCameraWidthSize = 10;
/// Number of channels for dummy camera
extern const uint8_t kChannelSize = 3;
/// Upper limit of camera data
extern const uint8_t kMaxCameraData = 255;
/// Diagnostics buffer size
extern const uint8_t kDiagBufferSize = 6;
/// rosout buffer size
extern const uint8_t kLogBufferSize = 100;


namespace camera_diag_updater {
/// @class CameraDiagComponentTest
/// @brief Base class for camera diagnostics tests
class CameraDiagComponentTest : public testing::Test {
 public:
  explicit CameraDiagComponentTest(std::string hardware_id)
    : diag_buffer_(kDiagBufferSize),
      rosout_msg_buffer_(kLogBufferSize),
      hardware_id_(hardware_id),
      publish_hz_(0.0),
      num_of_data_pub_(0) {
    node_ = rclcpp::Node::make_shared("camera_diag_updater");

    diag_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        kDiagTopicName, kDiagBufferSize,
        std::bind(&CameraDiagComponentTest::DiagCallback, this, std::placeholders::_1));

    rosout_sub_ = node_->create_subscription<rcl_interfaces::msg::Log>(
        "rosout", kLogBufferSize,
        std::bind(&CameraDiagComponentTest::RosoutCallback, this, std::placeholders::_1));

    // Wait until diagnostics can be retrieved
    rclcpp::Rate r(1.0);
    while (rclcpp::ok() && diag_buffer_.empty()) {
      rclcpp::spin_some(node_);
      r.sleep();
    }
    // Do not wait for topics to appear in rosout as it depends on the ROS version
  }

  /// Destructor
  ~CameraDiagComponentTest() {}

  /// Wait until timeout is stored in diagnostics
  bool WaitForBootTimeout(double timeout) {
    rclcpp::Time wait_start = node_->now();
    while ((node_->now() - wait_start).seconds() < timeout) {
      rclcpp::spin_some(node_);
      rclcpp::sleep_for(std::chrono::milliseconds(50));
      if (CheckOutput(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Boot timeout")) {
        return true;
      }
    }
    return false;
  }

  /// Publish dummy camera data
  virtual void StartPublish() = 0;

  /// Set the publishing frequency for dummy camera data
  /// @param [in] hz Data publishing frequency [Hz]
  void SetPublishHz(double hz) { publish_hz_ = hz; }

  /// Set the number of times dummy camera data is published
  /// @param [in] num_of_data_pub Number of data publications [times]
  void SetNumOfDataPub(uint8_t num_of_data_pub) { num_of_data_pub_ = num_of_data_pub; }

  /// Wait after publishing dummy camera data
  /// @param [in] duration Dummy camera wait time [s]
  void WaitDuration(double duration) {
    rclcpp::Time init = node_->now();
    rclcpp::Time now = init;
    rclcpp::Rate loop_rate(publish_hz_);
    while ((now - init).seconds() < duration) {
      rclcpp::spin_some(node_);
      now = node_->now();
      loop_rate.sleep();
    }
  }
  /// Determine if diagnostics output matches the expected output
  /// @param [in] expected_level Expected status level
  /// @param [in] expected_str Expected status message
  bool CheckOutput(uint8_t expected_level, std::string expected_str) {
    for (auto diag : diag_buffer_) {
      for (auto status : diag->status) {
        if (status.hardware_id == hardware_id_) {
          if ((status.level == expected_level) &&
              (status.message == expected_str)) {
            return true;
          }
        }
      }
    }
    return false;
  }



 protected:
  /// ROS node
  rclcpp::Node::SharedPtr node_;

  /// Circular buffer to store diagnostics
  boost::circular_buffer<diagnostic_msgs::msg::DiagnosticArray::SharedPtr> diag_buffer_;

  /// Circular buffer to store rosout messages
  boost::circular_buffer<std::string> rosout_msg_buffer_;

  /// Dummy data publishing frequency
  double publish_hz_;

  /// Number of dummy data publications
  uint8_t num_of_data_pub_;

 private:
  /// Callback function to receive camera diagnostics
  /// @param [in] diag Subscribed diagnostic data
  void DiagCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr diag) {
    diag_buffer_.push_back(diag);
  }

  /// Callback function to receive rosout logs
  /// @param [in] log Subscribed rosout data
  void RosoutCallback(const rcl_interfaces::msg::Log::SharedPtr log) { rosout_msg_buffer_.push_back(log->msg); }

  /// Topic name
  std::string hardware_id_;

  /// Diagnostics subscriber
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;

  /// Subscriber for standard error output
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;
};

/// @class CameraImageDiagComponentTest
/// @brief Test fixture for Image-type camera diagnostic node
class CameraImageDiagComponentTest : public CameraDiagComponentTest {
 public:
  CameraImageDiagComponentTest() : CameraDiagComponentTest(kCameraImageHardwareID) {
    camera_image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(kCameraImageTopicName, 1);
  }

  ~CameraImageDiagComponentTest() {}

  void StartPublish() override {
    rclcpp::Time init = node_->now();
    rclcpp::Rate loop_rate(publish_hz_);
    diag_buffer_.clear();

    uint8_t pub_count = 0;
    while (pub_count < num_of_data_pub_) {
      camera_image_pub_->publish(image_data_);
      pub_count++;
      rclcpp::spin_some(node_);
      loop_rate.sleep();
    }
  }


  /// Determine if expected output is included in rosout log output
  /// @param [in] expected_str Expected output
  bool CheckLogOutput(std::string expected_str) {
    for (uint8_t i = 0; i < rosout_msg_buffer_.size(); i++) {
      if (rosout_msg_buffer_.at(i) == expected_str) return true;
    }
    return false;
  }

  void CreateBgr8UnicolorData(uint8_t color_b, uint8_t color_g, uint8_t color_r) {
    image_data_.height = kCameraHeightSize;
    image_data_.width = kCameraWidthSize;
    image_data_.encoding = "bgr8";
    image_data_.step = kCameraWidthSize * kChannelSize;
    image_data_.data.clear();
    for (int i = 0; i < image_data_.height * image_data_.step; i += image_data_.step / image_data_.width) {
      image_data_.data.push_back(color_b);
      image_data_.data.push_back(color_g);
      image_data_.data.push_back(color_r);
    }
  }

  void CreateNormalData(std::string encoding) {
    image_data_.height = kCameraHeightSize;
    image_data_.width = kCameraWidthSize;
    image_data_.encoding = encoding;
    image_data_.step = kCameraWidthSize * kChannelSize;
    image_data_.data.clear();
    uint32_t seed(0);
    for (uint32_t i = 0; i < image_data_.height * image_data_.step; i++) {
      image_data_.data.push_back(rand_r(&seed) / kMaxCameraData);
    }
  }

 private:
  /// Dummy camera publisher for image
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_image_pub_;

  // Dummy data for image
  sensor_msgs::msg::Image image_data_;
};

/// @class CameraDiagComponentTest
/// @brief Test fixture for PointCloud2-type camera diagnostic node
class CameraPointsDiagComponentTest : public CameraDiagComponentTest {
 public:
  CameraPointsDiagComponentTest() : CameraDiagComponentTest(kCameraPointsHardwareID) {
    camera_points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(kCameraPointsTopicName, 1);
  }

  ~CameraPointsDiagComponentTest() {}

  void StartPublish() override {
    rclcpp::Time init = node_->now();
    rclcpp::Time now = init;
    rclcpp::Rate loop_rate(publish_hz_);
    diag_buffer_.clear();

    uint8_t pub_count = 0;
    while (pub_count < num_of_data_pub_) {
      camera_points_pub_->publish(points_data_);
      pub_count++;
      rclcpp::spin_some(node_);
      loop_rate.sleep();
    }
  }

  void CreateNormalData() {
    points_data_.height = kCameraHeightSize;
    points_data_.width = kCameraWidthSize;
    points_data_.row_step = kCameraWidthSize * kChannelSize;
    points_data_.data.clear();
    uint32_t seed(0);
    for (uint32_t i = 0; i < points_data_.height * points_data_.row_step; i++) {
      points_data_.data.push_back(rand_r(&seed) / kMaxCameraData);
    }
  }

 private:
  /// Dummy camera publisher for points
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr camera_points_pub_;

  /// Dummy data for points
  sensor_msgs::msg::PointCloud2 points_data_;
};
}  // namespace camera_diag_updater

#endif  // CAMERA_DIAG_UPDATER_TEST_FIXTURE_HPP_
