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
/// @brief Test for camera diag update functionality

#include <map>
#include <string>
#include <vector>

#include "camera_diag_updater/camera_diag_component.hpp"
#include "test_fixture.hpp"

namespace {
/// Green R value
constexpr uint8_t kGreenR = 0;
/// Green G value
constexpr uint8_t kGreenG = 154;
/// Green B value
constexpr uint8_t kGreenB = 0;
}  // unnamed namespace


namespace camera_diag_updater {
TEST_F(CameraImageDiagComponentTest, InvalidParamTest) {
  // Invalid values are set for required parameters
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override<std::string>("topic_name", "wrong_name");
  node_options.append_parameter_override<std::string>("topic_type", "wrong_type");

  auto camera_diag = std::make_shared<CameraDiagComponent>(node_options);
  rclcpp::sleep_for(std::chrono::seconds(1));

  auto topic_infos = node_->get_subscriptions_info_by_topic("wrong_name");
  EXPECT_TRUE(topic_infos.empty());
}

// Test for sensor_msgs::Image type
// Since the diag publishing cycle is 10Hz, at least 0.2 seconds are needed to reflect the desired test scenario
// Subscribe to data with unsupported encoding
TEST_F(CameraImageDiagComponentTest, NonSupportEncodingTest) {
  CreateNormalData("yuv422");
  SetPublishHz(25.0);
  SetNumOfDataPub(10);  // Publish messages for 0.4 seconds (10/25.0=)
  StartPublish();
  EXPECT_TRUE(CheckLogOutput("Unicolor check only support rgb8 or bgr8 encoding. Stop checking."));
}

// No messages are subscribed
TEST_F(CameraImageDiagComponentTest, ImageSubscribedNoMessageTest) {
  // Subscribe to the topic once to distinguish from startup errors
  CreateNormalData("bgr8");
  SetPublishHz(25.0);
  SetNumOfDataPub(10);  // Publish messages for 0.4 seconds (10/25.0=)
  StartPublish();
  WaitDuration(2.0);    // No messages are published for 2.0 seconds
  EXPECT_TRUE(CheckOutput(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No New Messages"));
}

// Subscribe to only one message
TEST_F(CameraImageDiagComponentTest, ImageSubscribedOneMessageTest) {
  CreateNormalData("bgr8");
  SetPublishHz(25.0);
  SetNumOfDataPub(1);
  StartPublish();
  WaitDuration(1.0);   // After receiving one message, no messages are published for slightly less than 1.0 seconds
  EXPECT_TRUE(CheckOutput(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                          "Only one message is subscribed. Cannot calculate frequency."));
}

// An image with a solid green color is being published
TEST_F(CameraImageDiagComponentTest, UniColorTest) {
  CreateBgr8UnicolorData(kGreenB, kGreenG, kGreenR);
  SetPublishHz(25.0);
  SetNumOfDataPub(10);  // Publish messages for 0.4 seconds (10/25.0=)
  StartPublish();
  EXPECT_TRUE(CheckOutput(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Fill image with unicolor"));
}

// An image with a solid black color is being published
TEST_F(CameraImageDiagComponentTest, UniColorFalseTest) {
  CreateBgr8UnicolorData(0, 0, 0);
  SetPublishHz(25.0);
  SetNumOfDataPub(10);  // Publish messages for 0.4 seconds (10/25.0=)
  StartPublish();
  EXPECT_TRUE(CheckOutput(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK"));
}

// Message acquisition cycle is below 20Hz
TEST_F(CameraImageDiagComponentTest, ImageSlowPublishRateTest) {
  CreateNormalData("bgr8");
  SetPublishHz(10.0);
  SetNumOfDataPub(5);  // Publish messages for 0.5 seconds (5/10.0=)
  StartPublish();
  EXPECT_TRUE(CheckOutput(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Subscribing Rate is slow"));
}

// An image with a solid green color is being published at less than 20Hz
TEST_F(CameraImageDiagComponentTest, ImageSlowAndUnicolorTest) {
  CreateBgr8UnicolorData(kGreenB, kGreenG, kGreenR);
  SetPublishHz(10.0);
  SetNumOfDataPub(5);  // Publish messages for 0.5 seconds (5/10.0=)
  StartPublish();
  EXPECT_TRUE(CheckOutput(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Fill image with unicolor"));
}

// OK
TEST_F(CameraImageDiagComponentTest, ImageNormalTest) {
  CreateNormalData("bgr8");
  SetPublishHz(25.0);
  SetNumOfDataPub(10);  // Publish messages for 0.4 seconds (10/25.0=)
  StartPublish();
  EXPECT_TRUE(CheckOutput(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK"));
}

// Test for sensor_msgs::PointCloud2 type
// Since the diag publishing cycle is 1Hz, at least 2 seconds are needed to reflect the desired test scenario
// No messages are subscribed
TEST_F(CameraPointsDiagComponentTest, PointsSubscribedNoMessageTest) {
  // Subscribe to the topic once to distinguish from startup errors
  CreateNormalData();
  SetPublishHz(15.0);
  SetNumOfDataPub(6);  // Publish messages for 0.4 seconds (6/15.0=)
  StartPublish();
  WaitDuration(2.0);  // No messages are published for 2.0 seconds
  EXPECT_TRUE(CheckOutput(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No New Messages"));
}

// Subscribe to only one message
TEST_F(CameraPointsDiagComponentTest, PointSubscribedOneMessageTest) {
  CreateNormalData();
  SetPublishHz(15.0);
  SetNumOfDataPub(1);
  StartPublish();
  WaitDuration(1.0);  // After receiving one message, no messages are published for slightly less than 1.0 seconds
  EXPECT_TRUE(CheckOutput(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                          "Only one message is subscribed. Cannot calculate frequency."));
}

// Message acquisition cycle is below 10Hz
TEST_F(CameraPointsDiagComponentTest, PointsSlowPublishRateTest) {
  CreateNormalData();
  SetPublishHz(5.0);
  SetNumOfDataPub(10);  // Publish messages for 2.0 seconds (10/5.0=)
  StartPublish();
  EXPECT_TRUE(CheckOutput(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Subscribing Rate is slow"));
}

// OK
TEST_F(CameraPointsDiagComponentTest, PointsNormalTest) {
  CreateNormalData();
  SetPublishHz(15.0);
  SetNumOfDataPub(15);  // Publish messages for 1.0 second (15/15.0=)
  StartPublish();
  EXPECT_TRUE(CheckOutput(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK"));
}
}  // namespace camera_diag_updater

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
