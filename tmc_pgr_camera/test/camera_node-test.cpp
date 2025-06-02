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
/// @brief      Test for camera node
#include <string>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <cv_bridge/cv_bridge.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "tmc_vision_msgs/srv/control_camera.hpp"

namespace {

// Topic name for the left image
const char* kLeftImageTopicName = "/stereo_camera/left/image_raw";

// Service name to control the camera
const char* kControlCameraServiceName = "control_camera";

// Service name for dynamic_reconfigure
const char* kDynamicReconfigureServiceName = "/stereo_camera/property/set_parameters";

// Name of the node used in the test
const char* kTestNodeName = "/stereo_camera";

// Parameter name to get camera settings
const char* kSettingParamName = "setting";

// Parameter name for setting camera properties
const char* const kPropertyParamName = "property";

// Camera setting parameter name for dynamic_reconfigure callback test
const char* const kTestDynamicReconfigureCallbackSettingName = "test_dynamic_reconfigure_callback_setting";

// Timeout
const float kTimeout = 5.0;
}  // anonymous namespace

namespace tmc_pgr_camera {

/// @brief Test fixture for camera node
class CameraNodeTest : public testing::Test {
 protected:
  /// @brief Subscriber initialization
  virtual void SetUp() {
    node_ = rclcpp::Node::make_shared("camera_node_test");
    s_images.set_capacity(3);
    s_image_sub = node_->create_subscription<sensor_msgs::msg::Image>(
      kLeftImageTopicName, 1, std::bind(&CameraNodeTest::ImageCallback, this, std::placeholders::_1));
  }

  virtual void TearDown() {
    node_.reset();
  }

  /// @brief Callback function to receive delivered images
  void ImageCallback(const sensor_msgs::msg::Image& image) { s_images.push_back(image); }

  inline size_t image_buffer_size() { return s_images.size(); }
  inline bool image_buffer_empty() { return s_images.empty(); }
  inline void clear_image_buffer() { s_images.clear(); }
  inline rclcpp::Node::SharedPtr get_node_handle() { return node_; }

 private:
  rclcpp::Node::SharedPtr node_;

  /// Image subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr s_image_sub;

  /// Ring buffer for received images
  boost::circular_buffer<sensor_msgs::msg::Image> s_images;
};

/// @brief Check if the camera node can process normally until delivery
TEST_F(CameraNodeTest, CheckPublishing) {
  const rclcpp::Time start_time = get_node_handle()->get_clock()->now();
  rclcpp::Rate rate(1);
  while (rclcpp::ok()) {
    rclcpp::spin_some(get_node_handle());
    rate.sleep();
    const rclcpp::Time current_time = get_node_handle()->get_clock()->now();
    if (current_time - start_time > rclcpp::Duration::from_seconds(kTimeout)) {
      break;
    }
  }
  ASSERT_NE(0u, image_buffer_size());
}

/// @brief Verify if the start/stop capture service works
TEST_F(CameraNodeTest, CheckService) {
  rclcpp::Client<tmc_vision_msgs::srv::ControlCamera>::SharedPtr client =
    get_node_handle()->create_client<tmc_vision_msgs::srv::ControlCamera>(kControlCameraServiceName);

  auto req = std::make_shared<tmc_vision_msgs::srv::ControlCamera::Request>();

  client->wait_for_service(std::chrono::milliseconds(static_cast<int>(kTimeout * 1000.0)));

  // Running → Stopped
  req->capture = false;
  auto res = client->async_send_request(req);
  auto res_service = rclcpp::spin_until_future_complete(get_node_handle(), res);
  ASSERT_TRUE(res_service == rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(res.get()->is_success);
  do {
    clear_image_buffer();
    rclcpp::spin_some(get_node_handle());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } while (!image_buffer_empty());
  rclcpp::spin_some(get_node_handle());
  ASSERT_EQ(0, image_buffer_size());

  // Stop command while stopped
  res = client->async_send_request(req);
  res_service = rclcpp::spin_until_future_complete(get_node_handle(), res);
  ASSERT_TRUE(res_service == rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_FALSE(res.get()->is_success);

  // Stopped → Started
  req->capture = true;
  res = client->async_send_request(req);
  res_service = rclcpp::spin_until_future_complete(get_node_handle(), res);
  ASSERT_TRUE(res_service == rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(res.get()->is_success);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(get_node_handle());
  ASSERT_LT(0, image_buffer_size());

  // Start command while running
  res = client->async_send_request(req);
  res_service = rclcpp::spin_until_future_complete(get_node_handle(), res);
  ASSERT_TRUE(res_service == rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_FALSE(res.get()->is_success);
}

/// @brief Parameter setting confirmation
TEST_F(CameraNodeTest, CheckDynamicReconfigure) {
  auto param_client =
    std::make_shared<rclcpp::SyncParametersClient>(get_node_handle(), kTestNodeName);

  param_client->wait_for_service(std::chrono::milliseconds(static_cast<int>(kTimeout * 1000.0)));

  std::string ns = std::string(kPropertyParamName);

  // Check if the parameter server is running
  ASSERT_TRUE(param_client->service_is_ready());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(get_node_handle());

  // Pass parameter directly
  std::vector<rclcpp::Parameter> setting_params = {
    // brightness
    rclcpp::Parameter(ns + std::string(".brightness_abs_value"), 1.367188),
    // auto_exposure
    rclcpp::Parameter(ns + std::string(".auto_exposure_one_push"), true),
    rclcpp::Parameter(ns + std::string(".auto_exposure_on_off"), true),
    rclcpp::Parameter(ns + std::string(".auto_exposure_auto_manual_mode"), true),
    rclcpp::Parameter(ns + std::string(".auto_exposure_abs_value"), 0.0),
    // white_balance
    rclcpp::Parameter(ns + std::string(".white_balance_one_push"), true),
    rclcpp::Parameter(ns + std::string(".white_balance_on_off"), true),
    rclcpp::Parameter(ns + std::string(".white_balance_auto_manual_mode"), true),
    rclcpp::Parameter(ns + std::string(".white_balance_value_a"), 570),
    rclcpp::Parameter(ns + std::string(".white_balance_value_b"), 810),
    // shutter
    rclcpp::Parameter(ns + std::string(".shutter_one_push"), false),
    rclcpp::Parameter(ns + std::string(".shutter_auto_manual_mode"), false),
    rclcpp::Parameter(ns + std::string(".shutter_abs_value"), 20.0),
    // gain
    rclcpp::Parameter(ns + std::string(".gain_one_push"), false),
    rclcpp::Parameter(ns + std::string(".gain_auto_manual_mode"), false),
    rclcpp::Parameter(ns + std::string(".gain_abs_value"), 0.0),
    // trigger_mode
    rclcpp::Parameter(ns + std::string(".trigger_mode_polarity"), 0),
    rclcpp::Parameter(ns + std::string(".trigger_mode_on_off"), false),
    rclcpp::Parameter(ns + std::string(".trigger_mode_mode"), 0),
    // trigger_delay
    rclcpp::Parameter(ns + std::string(".trigger_delay_abs_control"), true),
    rclcpp::Parameter(ns + std::string(".trigger_delay_on_off"), false),
    rclcpp::Parameter(ns + std::string(".trigger_delay_value_a"), 307),
    rclcpp::Parameter(ns + std::string(".trigger_delay_abs_value"), 5.0),
    // frame_rate
    rclcpp::Parameter(ns + std::string(".frame_rate_abs_control"), false),
    rclcpp::Parameter(ns + std::string(".frame_rate_on_off"), false),
    rclcpp::Parameter(ns + std::string(".frame_rate_auto_manual_mode"), false),
    rclcpp::Parameter(ns + std::string(".frame_rate_value_a"), 480),
    rclcpp::Parameter(ns + std::string(".frame_rate_abs_value"), 5.0),
  };

  auto set_parameters_results = param_client->set_parameters(setting_params);
  for (auto & res : set_parameters_results) {
    ASSERT_TRUE(res.successful);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(get_node_handle());

  // Check if the one_push setting is cleared
  std::vector<rclcpp::Parameter> one_push_params = {
    rclcpp::Parameter(ns + std::string(".auto_exposure_one_push"), true),
    rclcpp::Parameter(ns + std::string(".white_balance_one_push"), true),
    rclcpp::Parameter(ns + std::string(".shutter_one_push"), true),
    rclcpp::Parameter(ns + std::string(".gain_one_push"), true),
  };
  auto one_push_parameters_result = param_client->set_parameters(one_push_params);
  for (auto & res : one_push_parameters_result) {
    ASSERT_TRUE(res.successful);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(get_node_handle());
  // One_push changes to false after the camera settings are applied
  for (auto & param : param_client->get_parameters({
    ns + std::string(".auto_exposure_one_push"),
    ns + std::string(".white_balance_one_push"),
    ns + std::string(".shutter_one_push"),
    ns + std::string(".gain_one_push"),
  })) {
    ASSERT_FALSE(param.get_value<bool>());
  }

  // Confirm that setting mode to 1 in trigger_mode makes auto_manual_mode for shutter unable to be set.
  auto trigger_mode_1_parameters_result = param_client->set_parameters({
    rclcpp::Parameter(ns + std::string(".trigger_mode_mode"), 1),
    rclcpp::Parameter(ns + std::string(".shutter_auto_manual_mode"), true),
  });
  for (auto & res : trigger_mode_1_parameters_result) {
    ASSERT_TRUE(res.successful);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(get_node_handle());
  // shutter_auto_manual_mode is forcibly changed to false when trigger_mode_mode is 1
  ASSERT_FALSE(param_client->get_parameter<bool>(ns + std::string(".shutter_auto_manual_mode")));

  // Confirm that auto_manual_mode for shutter can be set when mode in trigger_mode is set to anything other than 1.
  auto trigger_mode_0_parameters_result = param_client->set_parameters({
    rclcpp::Parameter(ns + std::string(".trigger_mode_mode"), 0),
    rclcpp::Parameter(ns + std::string(".shutter_auto_manual_mode"), true),
  });
  for (auto & res : trigger_mode_0_parameters_result) {
    ASSERT_TRUE(res.successful);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(get_node_handle());
  // shutter_auto_manual_mode is retained if trigger_mode_mode is not 1
  ASSERT_TRUE(param_client->get_parameter<bool>(ns + std::string(".shutter_auto_manual_mode")));

  // Confirm that setting on_off to on (1) in trigger_mode disables frame_rate setting.
  auto trigger_mode_on_off_1_parameters_result = param_client->set_parameters({
    rclcpp::Parameter(ns + std::string(".trigger_mode_on_off"), true),
    rclcpp::Parameter(ns + std::string(".frame_rate_on_off"), true),
    rclcpp::Parameter(ns + std::string(".frame_rate_auto_manual_mode"), true),
  });
  for (auto & res : trigger_mode_on_off_1_parameters_result) {
    ASSERT_TRUE(res.successful);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(get_node_handle());
  // frame_rate related settings are forcibly changed to false
  ASSERT_FALSE(param_client->get_parameter<bool>(ns + std::string(".frame_rate_on_off")));
  ASSERT_FALSE(param_client->get_parameter<bool>(ns + std::string(".frame_rate_auto_manual_mode")));

  // Confirm that setting on_off to off (0) in trigger_mode allows frame_rate setting.
  auto trigger_mode_on_off_0_parameters_result = param_client->set_parameters({
    rclcpp::Parameter(ns + std::string(".trigger_mode_on_off"), false),
    rclcpp::Parameter(ns + std::string(".frame_rate_on_off"), true),
    rclcpp::Parameter(ns + std::string(".frame_rate_auto_manual_mode"), true),
  });
  for (auto & res : trigger_mode_on_off_0_parameters_result) {
    ASSERT_TRUE(res.successful);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(get_node_handle());
  // frame_rate related settings are retained
  ASSERT_TRUE(param_client->get_parameter<bool>(ns + std::string(".frame_rate_on_off")));
  ASSERT_TRUE(param_client->get_parameter<bool>(ns + std::string(".frame_rate_auto_manual_mode")));
}

}  // end of namespace tmc_pgr_camera

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
