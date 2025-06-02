/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
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
#include <string>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include "test_utils.hpp"

namespace {
const double kPermissibleRateError = 3.0;
}

namespace tmc_diag_updater_common {

class DiagUpdaterNodeTest : public testing::Test {
 public:
  DiagUpdaterNodeTest() { node_ = rclcpp::Node::make_shared("diag_updater_node_test"); }

  virtual ~DiagUpdaterNodeTest() = default;

  // Process at the start of each test
  void SetUp() override {
    imu_pub_.reset(new test_utils::CyclicImuPublisher(node_));
    diag_sub_.reset(new test_utils::DiagCacheSubscriber(node_));

    // It's cumbersome to bind every time, so do it here
    imu_is_subscribed_ = [this]() { return imu_pub_->ImuIsSubscribed(); };
    diag_is_advertised_ = [this]() { return diag_sub_->IsDiagMessageAvailable(); };
    cache_length_greater_than_3_ = [this]() { return diag_sub_->GetQueueLength() > 3; };
  }

  // Process at the end of each test
  // - Publish completion
  // - Subscription completion
  void TearDown() override {
    imu_pub_.reset();

    // Pray for disconnection
    std::function<bool()> disconnected = [this] {
      return diag_sub_->GetLatestMessage().status[0].message.find("Disconnected") != std::string::npos;
    };
    test_utils::WaitUntil(node_, disconnected, 3.0);

    diag_sub_.reset();
  }

 protected:
  // Send/receive confirmation utilities
  std::function<bool()> imu_is_subscribed_;
  std::function<bool()> diag_is_advertised_;
  std::function<bool()> cache_length_greater_than_3_;

  // I/O
  test_utils::CyclicImuPublisher::SharedPtr imu_pub_;
  test_utils::DiagCacheSubscriber::SharedPtr diag_sub_;
  rclcpp::Node::SharedPtr node_;
};
// Initially, notify that there are no topics
TEST_F(DiagUpdaterNodeTest, DefaultDiag) {
  // Setup
  // None

  // Exercise
  diag_sub_->StartCaching();
  test_utils::WaitUntilTimuout(node_, 3.0);
  ASSERT_TRUE(test_utils::WaitUntil(node_, diag_is_advertised_, 1.0));
  ASSERT_TRUE(test_utils::WaitUntil(node_, imu_is_subscribed_, 1.0));
  ASSERT_TRUE(test_utils::WaitUntil(node_, cache_length_greater_than_3_, 10.0));

  diagnostic_msgs::msg::DiagnosticArray diag_msg = diag_sub_->GetLatestMessage();

  // Verify
  RCLCPP_ERROR_STREAM(node_->get_logger(), "status" << diag_msg.status[0].message);
  EXPECT_EQ(diag_msg.status.size(), 1);
  EXPECT_EQ(diag_msg.status[0].level, 2);
  EXPECT_STREQ(diag_msg.status[0].name.c_str(), "imu_diag_updater: imu topic status");
  EXPECT_TRUE(diag_msg.status[0].message.find("Disconnected") != std::string::npos);
  EXPECT_STREQ(diag_msg.status[0].hardware_id.c_str(), "imu");
}
// Expected to get OK when outputting the expected Imu data
TEST_F(DiagUpdaterNodeTest, GetOkDiag) {
  // Setup
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "base_imu_frame";
  imu_msg.linear_acceleration.z = -9.8;
  imu_pub_->set_imu_msg(imu_msg);
  constexpr double publish_hz = 100.0;

  // Exercise
  ASSERT_TRUE(test_utils::WaitUntil(node_, imu_is_subscribed_, 3.0));
  imu_pub_->StartPublishing<test_utils::DummySensorTimeForward>(node_->get_clock(), publish_hz);
  diag_sub_->StartCaching();
  test_utils::WaitUntilTimuout(node_, 1.0);

  ASSERT_TRUE(test_utils::WaitUntil(node_, diag_is_advertised_, 3.0));
  ASSERT_TRUE(test_utils::WaitUntil(node_, cache_length_greater_than_3_, 5.0));
  diag_sub_->StopCaching();

  // Verify
  const diagnostic_msgs::msg::DiagnosticArray diag_msg = diag_sub_->GetLatestMessage();
  EXPECT_EQ(diag_msg.status.size(), 1);
  EXPECT_EQ(diag_msg.status[0].level, 0);
  EXPECT_STREQ(diag_msg.status[0].name.c_str(), "imu_diag_updater: imu topic status");
  EXPECT_STREQ(diag_msg.status[0].message.c_str(), "OK");
  EXPECT_STREQ(diag_msg.status[0].hardware_id.c_str(), "imu");
  const double diff_hz =
      std::abs(test_utils::PickUpDiagValueOf("Actual rate (Hz)", diag_msg.status[0].values) - publish_hz);
  EXPECT_LT(diff_hz, kPermissibleRateError);
}
// Abnormal Diag should be returned if frame_id is incorrect
// Phenomenon confirmed in tmc_adi_driver
TEST_F(DiagUpdaterNodeTest, GetUnExpectedFrameIdError) {
  // Setup
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "!!binary |\n/8ZkXJq+PRJ1cmF0ZV9pbXVfZnJhbWu=";
  imu_msg.linear_acceleration.z = -9.8;
  imu_pub_->set_imu_msg(imu_msg);
  const double publish_hz = 100.0;

  // Exercise
  ASSERT_TRUE(test_utils::WaitUntil(node_, imu_is_subscribed_, 3.0));
  imu_pub_->StartPublishing<test_utils::DummySensorTimeForward>(node_->get_clock(), publish_hz);
  diag_sub_->StartCaching();
  ASSERT_TRUE(test_utils::WaitUntil(node_, diag_is_advertised_, 3.0));
  ASSERT_TRUE(test_utils::WaitUntil(node_, cache_length_greater_than_3_, 5.0));
  diag_sub_->StopCaching();

  // Verify
  const diagnostic_msgs::msg::DiagnosticArray diag_msg = diag_sub_->GetLatestMessage();
  EXPECT_EQ(diag_msg.status.size(), 1);
  EXPECT_EQ(diag_msg.status[0].level, 2);
  EXPECT_STREQ(diag_msg.status[0].name.c_str(), "imu_diag_updater: imu topic status");
  EXPECT_TRUE(diag_msg.status[0].message.find("Unexpected frame ID") != std::string::npos);
  EXPECT_STREQ(diag_msg.status[0].hardware_id.c_str(), "imu");
}

// Send IMU topic at slightly lower than expected frequency, should result in WARN
// Expected to return WARN level
TEST_F(DiagUpdaterNodeTest, GetSlightlyLowRatencyWarn) {
  // Setup
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "base_imu_frame";
  imu_msg.linear_acceleration.z = -9.8;
  imu_pub_->set_imu_msg(imu_msg);
  const double publish_hz = 40.0;

  // Exercise
  ASSERT_TRUE(test_utils::WaitUntil(node_, imu_is_subscribed_, 3.0));
  imu_pub_->StartPublishing<test_utils::DummySensorTimeForward>(node_->get_clock(), publish_hz);
  diag_sub_->StartCaching();
  test_utils::WaitUntilTimuout(node_, 1.0);
  ASSERT_TRUE(test_utils::WaitUntil(node_, diag_is_advertised_, 3.0));
  ASSERT_TRUE(test_utils::WaitUntil(node_, cache_length_greater_than_3_, 5.0));
  diag_sub_->StopCaching();

  // Verify
  const diagnostic_msgs::msg::DiagnosticArray diag_msg = diag_sub_->GetLatestMessage();
  EXPECT_EQ(diag_msg.status.size(), 1);
  EXPECT_EQ(diag_msg.status[0].level, 1);
  EXPECT_STREQ(diag_msg.status[0].name.c_str(), "imu_diag_updater: imu topic status");
  EXPECT_STREQ(diag_msg.status[0].message.c_str(), "Slightly low rate");
  EXPECT_STREQ(diag_msg.status[0].hardware_id.c_str(), "imu");
  const double diff_hz =
      std::abs(test_utils::PickUpDiagValueOf("Actual rate (Hz)", diag_msg.status[0].values) - publish_hz);
  EXPECT_LT(diff_hz, kPermissibleRateError);
}

// Send IMU topic at even lower than the above test, should result in ERROR
// Expected to return ERROR level
TEST_F(DiagUpdaterNodeTest, GetSignificantlyLowRatencyError) {
  // Setup
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "base_imu_frame";
  imu_msg.linear_acceleration.z = -9.8;
  imu_pub_->set_imu_msg(imu_msg);
  const double publish_hz = 20.0;

  // Exercise
  ASSERT_TRUE(test_utils::WaitUntil(node_, imu_is_subscribed_, 3.0));
  imu_pub_->StartPublishing<test_utils::DummySensorTimeForward>(node_->get_clock(), publish_hz);
  diag_sub_->StartCaching();
  test_utils::WaitUntilTimuout(node_, 3.0);
  ASSERT_TRUE(test_utils::WaitUntil(node_, diag_is_advertised_, 3.0));
  ASSERT_TRUE(test_utils::WaitUntil(node_, cache_length_greater_than_3_, 5.0));
  diag_sub_->StopCaching();

  // Verify
  const diagnostic_msgs::msg::DiagnosticArray diag_msg = diag_sub_->GetLatestMessage();
  EXPECT_EQ(diag_msg.status.size(), 1);
  EXPECT_EQ(diag_msg.status[0].level, 2);
  EXPECT_STREQ(diag_msg.status[0].name.c_str(), "imu_diag_updater: imu topic status");
  EXPECT_STREQ(diag_msg.status[0].message.c_str(), "Significantly low rate");
  EXPECT_STREQ(diag_msg.status[0].hardware_id.c_str(), "imu");
  const double diff_hz =
      std::abs(test_utils::PickUpDiagValueOf("Actual rate (Hz)", diag_msg.status[0].values) - publish_hz);
  EXPECT_LT(diff_hz, kPermissibleRateError);
}

// If the flow of time reverses, should result in ERROR
TEST_F(DiagUpdaterNodeTest, GetPastTimestamp) {
  // Setup
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "base_imu_frame";
  imu_msg.linear_acceleration.z = -9.8;
  imu_pub_->set_imu_msg(imu_msg);
  const double publish_hz = 100.0;

  // Exercise
  ASSERT_TRUE(test_utils::WaitUntil(node_, imu_is_subscribed_, 3.0));
  imu_pub_->StartPublishing<test_utils::DummySensorTimeBackward>(node_->get_clock(), publish_hz);
  diag_sub_->StartCaching();
  test_utils::WaitUntilTimuout(node_, 3.0);
  ASSERT_TRUE(test_utils::WaitUntil(node_, diag_is_advertised_, 3.0));
  ASSERT_TRUE(test_utils::WaitUntil(node_, cache_length_greater_than_3_, 5.0));
  diag_sub_->StopCaching();

  // Verify
  const diagnostic_msgs::msg::DiagnosticArray diag_msg = diag_sub_->GetLatestMessage();
  EXPECT_EQ(diag_msg.status.size(), 1);
  EXPECT_EQ(diag_msg.status[0].level, 2);
  EXPECT_STREQ(diag_msg.status[0].name.c_str(), "imu_diag_updater: imu topic status");
  EXPECT_STREQ(diag_msg.status[0].message.c_str(), "The same or past timestamp");
  EXPECT_STREQ(diag_msg.status[0].hardware_id.c_str(), "imu");
}

// If the flow of time stops, should result in ERROR
TEST_F(DiagUpdaterNodeTest, GetTheSameTimestamp) {
  // Setup
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "base_imu_frame";
  imu_msg.linear_acceleration.z = -9.8;
  imu_pub_->set_imu_msg(imu_msg);
  const double publish_hz = 100.0;

  // Exercise
  ASSERT_TRUE(test_utils::WaitUntil(node_, imu_is_subscribed_, 3.0));
  imu_pub_->StartPublishing<test_utils::DummySensorTimeFreezed>(node_->get_clock(), publish_hz);
  diag_sub_->StartCaching();
  test_utils::WaitUntilTimuout(node_, 3.0);
  ASSERT_TRUE(test_utils::WaitUntil(node_, diag_is_advertised_, 3.0));
  ASSERT_TRUE(test_utils::WaitUntil(node_, cache_length_greater_than_3_, 5.0));
  diag_sub_->StopCaching();

  // Verify
  const diagnostic_msgs::msg::DiagnosticArray diag_msg = diag_sub_->GetLatestMessage();
  EXPECT_EQ(diag_msg.status.size(), 1);
  EXPECT_EQ(diag_msg.status[0].level, 2);
  EXPECT_STREQ(diag_msg.status[0].name.c_str(), "imu_diag_updater: imu topic status");
  EXPECT_STREQ(diag_msg.status[0].message.c_str(), "The same or past timestamp");
  EXPECT_STREQ(diag_msg.status[0].hardware_id.c_str(), "imu");
}

// Expected to return if multiple Diags of the same level occur
TEST_F(DiagUpdaterNodeTest, GetMultiErrorDiag) {
  // Setup
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "!!binary |\n/8ZkXJq+PRJ1cmF0ZV9pbXVfZnJhbWu=";
  imu_msg.linear_acceleration.z = -9.8;
  imu_pub_->set_imu_msg(imu_msg);
  const double publish_hz = 20.0;

  // Exercise
  ASSERT_TRUE(test_utils::WaitUntil(node_, imu_is_subscribed_, 3.0));
  imu_pub_->StartPublishing<test_utils::DummySensorTimeForward>(node_->get_clock(), publish_hz);
  diag_sub_->StartCaching();
  test_utils::WaitUntilTimuout(node_, 3.0);
  ASSERT_TRUE(test_utils::WaitUntil(node_, diag_is_advertised_, 3.0));
  ASSERT_TRUE(test_utils::WaitUntil(node_, cache_length_greater_than_3_, 5.0));
  diag_sub_->StopCaching();

  // Verify
  const diagnostic_msgs::msg::DiagnosticArray diag_msg = diag_sub_->GetLatestMessage();
  EXPECT_EQ(diag_msg.status.size(), 1);
  EXPECT_EQ(diag_msg.status[0].level, 2);
  EXPECT_STREQ(diag_msg.status[0].name.c_str(), "imu_diag_updater: imu topic status");
  EXPECT_TRUE(diag_msg.status[0].message.find("Significantly low rate") != std::string::npos);
  EXPECT_TRUE(diag_msg.status[0].message.find("Unexpected frame ID") != std::string::npos);
  EXPECT_STREQ(diag_msg.status[0].hardware_id.c_str(), "imu");
}

// If WARN and ERROR occur simultaneously, ERROR should take precedence
//
// - Priority by level
//    OK < WARN < ERROR
//
// Among the following two intentionally caused Diags, Unexpected frame ID ERROR, which is ERROR, takes precedence and the level should be ERROR
// - Unexpected frame ID(ERROR)
// - Slightly low rate(WARN)
// Both messages are displayed
TEST_F(DiagUpdaterNodeTest, GetErrorShovePastWARN) {
  // Setup
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "!!binary |\n/8ZkXJq+PRJ1cmF0ZV9pbXVfZnJhbWu=";
  imu_msg.linear_acceleration.z = -9.8;
  imu_pub_->set_imu_msg(imu_msg);
  const double publish_hz = 40.0;

  // Exercise
  ASSERT_TRUE(test_utils::WaitUntil(node_, imu_is_subscribed_, 3.0));
  imu_pub_->StartPublishing<test_utils::DummySensorTimeForward>(node_->get_clock(), publish_hz);
  diag_sub_->StartCaching();
  test_utils::WaitUntilTimuout(node_, 3.0);
  ASSERT_TRUE(test_utils::WaitUntil(node_, diag_is_advertised_, 3.0));
  ASSERT_TRUE(test_utils::WaitUntil(node_, cache_length_greater_than_3_, 5.0));
  diag_sub_->StopCaching();

  // Verify
  const diagnostic_msgs::msg::DiagnosticArray diag_msg = diag_sub_->GetLatestMessage();
  EXPECT_EQ(diag_msg.status.size(), 1);
  EXPECT_EQ(diag_msg.status[0].level, 2);
  EXPECT_STREQ(diag_msg.status[0].name.c_str(), "imu_diag_updater: imu topic status");
  EXPECT_TRUE(diag_msg.status[0].message.find("Slightly low rate") != std::string::npos);
  EXPECT_TRUE(diag_msg.status[0].message.find("Unexpected frame ID") != std::string::npos);
  EXPECT_STREQ(diag_msg.status[0].hardware_id.c_str(), "imu");
}
}  // namespace tmc_diag_updater_common


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
