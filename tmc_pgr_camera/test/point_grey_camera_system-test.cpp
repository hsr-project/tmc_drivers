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
/// @brief Test of the Point Grey camera system
#include <filesystem>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <gtest/gtest.h>

#include "tmc_pgr_camera/point_grey_camera_system.hpp"
#include "tmc_pgr_camera/yaml_point_grey_camera_system_setting.hpp"


namespace {

// Camera configuration file
const char* kCameraSystemSettingFileName = "test/config/default.yml";

// Configuration file without self-trigger settings even though self-trigger is ON
const char* kMissingSelfTrigerSettingFileName = "test/config/get_trigger_mode_abnormal_00.yml";
}

namespace tmc_pgr_camera {

/// @brief Test fixture for the Point Grey camera system
class PointGreyCameraSystemTest : public testing::Test {
 protected:
  /// Initialization of the camera system
  void SetUp() {
    ASSERT_NO_THROW(camera_setting_.reset(
      new YamlPointGreyCameraSystemSetting(kCameraSystemSettingFileName)));
    ASSERT_NO_THROW(camera_system_.reset(new PointGreyCameraSystem(camera_setting_)));
  }

  /// Camera system
  std::shared_ptr<PointGreyCameraSystem> camera_system_;
  std::shared_ptr<IPointGreyCameraSystemSetting> camera_setting_;
};

/// @brief Whether the camera system starts up correctly
TEST_F(PointGreyCameraSystemTest, LaunchCameraSystemNormal) {
  ASSERT_NO_THROW(camera_setting_.reset(
    new YamlPointGreyCameraSystemSetting(kCameraSystemSettingFileName)));
  ASSERT_NO_THROW(PointGreyCameraSystem camera(camera_setting_));
  ASSERT_FALSE(camera_system_->IsOpened());
  ASSERT_NO_THROW(camera_system_->Open());
  ASSERT_TRUE(camera_system_->IsOpened());

  // No error occurs even when called twice
  ASSERT_NO_THROW(camera_system_->Open());
}

/// @brief Camera system does not start up correctly
TEST_F(PointGreyCameraSystemTest, LaunchCameraSystemAbnormal) {
  // Configuration is not specified even though self-trigger is ON
  ASSERT_TRUE(std::filesystem::exists(
      kMissingSelfTrigerSettingFileName));
  ASSERT_NO_THROW(camera_setting_.reset(
      new YamlPointGreyCameraSystemSetting(kMissingSelfTrigerSettingFileName)));
  ASSERT_ANY_THROW(camera_system_.reset(new PointGreyCameraSystem(camera_setting_)));
}

/// @brief Capture can be started
TEST_F(PointGreyCameraSystemTest, StartCaptureNormal) {
  ASSERT_NO_THROW(camera_system_->Open());
  ASSERT_FALSE(camera_system_->IsCapturing());
  ASSERT_NO_THROW(camera_system_->StartCapture());
  ASSERT_TRUE(camera_system_->IsCapturing());

  // No error occurs even when called twice
  // Standard error output indicates that it is already being captured
  std::stringbuf buf;
  std::streambuf* prev = std::cerr.rdbuf(&buf);
  ASSERT_NO_THROW(camera_system_->StartCapture());
  std::cerr.rdbuf(prev);
  // Extract the first line of console_bridge
  std::list<std::string> words;
  std::string delim("\n");
  std::string log = buf.str();
  boost::split(words, log, boost::is_any_of(delim));
  ASSERT_EQ("Warning: Already start capture.", *words.begin());
}

/// @brief Capture can be stopped
TEST_F(PointGreyCameraSystemTest, StopCaptureNormal) {
  ASSERT_NO_THROW(camera_system_->Open());

  // No error is thrown even when called in a non-capturing state
  ASSERT_NO_THROW(camera_system_->StopCapture());

  // Capture ends
  ASSERT_NO_THROW(camera_system_->StartCapture());
  ASSERT_TRUE(camera_system_->IsCapturing());
  ASSERT_NO_THROW(camera_system_->StopCapture());
  ASSERT_FALSE(camera_system_->IsCapturing());
}

/// @brief Image can be obtained
TEST_F(PointGreyCameraSystemTest, GrabImageNormal) {
  ASSERT_NO_THROW(camera_system_->Open());
  ASSERT_NO_THROW(camera_system_->StartCapture());

  // Since it is a self-trigger setting, the timestamps of the two obtained images are the same
  std::optional<std::vector<ImagePtr> > images;
  ASSERT_NO_THROW(images = camera_system_->GrabImage());
  ASSERT_EQ(2, images->size());
  ASSERT_EQ(images->at(0)->time, images->at(1)->time);

  // Verification that the obtained image is correct
  ASSERT_EQ(960, images->at(0)->image.rows);
  ASSERT_EQ(1280, images->at(0)->image.cols);
  ASSERT_EQ(960, images->at(1)->image.rows);
  ASSERT_EQ(1280, images->at(1)->image.cols);
}

/// @brief Verify if the configuration is successful
TEST_F(PointGreyCameraSystemTest, SetSettingNormal) {
  ASSERT_NO_THROW(camera_system_->Open());

  // Verify if the following can be configured
  // {
  //   property: [
  //     { type: brightness, absValue: 0.0, ..., onePush: on }
  //   ]
  // }
  YAML::Node brightness;
  brightness["type"] = std::string("brightness");
  brightness["present"] = true;
  brightness["absControl"] = true;
  brightness["onePush"] = true;
  brightness["onOff"] = true;
  brightness["autoManualMode"] = true;
  brightness["valueA"] = 0;
  brightness["valueB"] = 0;
  brightness["absValue"] = 0.0;
  YAML::Node properties;
  properties.push_back(brightness);
  YAML::Node setting;
  setting["property"] = properties;

  ASSERT_NO_THROW(camera_system_->SetSettings(setting));

  // Verify if trigger_mode can be configured
  YAML::Node trigger_mode;
  trigger_mode["type"] = std::string("trigger_mode");
  trigger_mode["mode"] = 0;
  trigger_mode["onOff"] = true;
  trigger_mode["polarity"] = 0;
  properties.reset();
  properties.push_back(trigger_mode);
  setting["property"] = properties;

  ASSERT_NO_THROW(camera_system_->SetSettings(setting));
}

/// @brief Configuration fails when the camera is not started
TEST_F(PointGreyCameraSystemTest, SetSettingWithoutOpenAbnormal) {
  YAML::Node setting;
  ASSERT_ANY_THROW(camera_system_->SetSettings(setting));
}

/// @brief Configuration fails with empty values
TEST_F(PointGreyCameraSystemTest, SetSettingWithEmptyValueAbnormal) {
  ASSERT_NO_THROW(camera_system_->Open());

  // Setting is not TypeStruct
  YAML::Node setting;
  ASSERT_ANY_THROW(camera_system_->SetSettings(setting));
}

/// @brief Configuration fails without specifying property type
TEST_F(PointGreyCameraSystemTest, SetSettingWithoutTypeAbnormal) {
  ASSERT_NO_THROW(camera_system_->Open());

  YAML::Node brightness;
  brightness["present"] = true;
  brightness["absControl"] = true;
  brightness["onePush"] = true;
  brightness["onOff"] = true;
  brightness["autoManualMode"] = true;
  brightness["valueA"] = 0;
  brightness["valueB"] = 0;
  brightness["absValue"] = 0.0;
  YAML::Node properties;
  properties.push_back(brightness);
  YAML::Node setting;
  setting["property"] = properties;

  ASSERT_ANY_THROW(camera_system_->SetSettings(setting));
}

/// @brief Configuration fails with an invalid property type
TEST_F(PointGreyCameraSystemTest, SetSettingWithInvalidTypeAbnormal) {
  ASSERT_NO_THROW(camera_system_->Open());

  YAML::Node brightness;
  brightness["type"] = "test";
  brightness["present"] = true;
  brightness["absControl"] = true;
  brightness["onePush"] = true;
  brightness["onOff"] = true;
  brightness["autoManualMode"] = true;
  brightness["valueA"] = 0;
  brightness["valueB"] = 0;
  brightness["absValue"] = 0.0;
  YAML::Node properties;
  properties.push_back(brightness);
  YAML::Node setting;
  setting["property"] = properties;

  ASSERT_ANY_THROW(camera_system_->SetSettings(setting));
}

/// @brief Configuration fails with an invalid value
/// Due to the change from XmlRpc to YAML, differences in numeric types are absorbed.
// TEST_F(PointGreyCameraSystemTest, SetSettingWithInvalidValueAbnormal) {
//   ASSERT_NO_THROW(camera_system_->Open());

//   YAML::Node brightness;
//   YAML::Node trigger_mode;
//   YAML::Node properties;
//   YAML::Node setting;
//   brightness["type"] = "brightness";
//   brightness["present"] = true;
//   brightness["absControl"] = true;
//   brightness["onePush"] = true;
//   brightness["onOff"] = true;
//   brightness["autoManualMode"] = true;
//   brightness["valueA"] = 0;
//   brightness["valueB"] = 0;
//   brightness["absValue"] = 0.0;
//   trigger_mode["type"] = "trigger_mode";
//   trigger_mode["mode"] = 0;
//   trigger_mode["onOff"] = true;
//   trigger_mode["polarity"] = 0;

//   // present is not boolean
//   brightness["present"] = 10;
//   properties.reset();
//   properties.push_back(brightness);
//   setting["property"] = properties;
//   ASSERT_ANY_THROW(camera_system_->SetSettings(setting));
//   brightness["present"] = true;

//   // absControl is not boolean
//   brightness["absControl"] = 10;
//   properties.reset();
//   properties.push_back(brightness);
//   setting["property"] = properties;
//   ASSERT_ANY_THROW(camera_system_->SetSettings(setting));
//   brightness["absControl"] = true;

//   // onePush is not boolean
//   brightness["onePush"] = 10;
//   properties.reset();
//   properties.push_back(brightness);
//   setting["property"] = properties;
//   ASSERT_ANY_THROW(camera_system_->SetSettings(setting));
//   brightness["onePush"] = true;

//   // onOff is not boolean
//   brightness["onOff"] = 10;
//   properties.reset();
//   properties.push_back(brightness);
//   setting["property"] = properties;
//   ASSERT_ANY_THROW(camera_system_->SetSettings(setting));
//   brightness["onOff"] = true;

//   // autoManualMode is not boolean
//   brightness["autoManualMode"] = 10;
//   properties.reset();
//   properties.push_back(brightness);
//   setting["property"] = properties;
//   ASSERT_ANY_THROW(camera_system_->SetSettings(setting));
//   brightness["autoManualMode"] = true;

//   // valueA is not int
//   brightness["valueA"] = 0.0;
//   properties.reset();
//   properties.push_back(brightness);
//   setting["property"] = properties;
//   ASSERT_ANY_THROW(camera_system_->SetSettings(setting));
//   brightness["valueA"] = 0;

//   // valueB is not int
//   brightness["valueB"] = 0.0;
//   properties.reset();
//   properties.push_back(brightness);
//   setting["property"] = properties;
//   ASSERT_ANY_THROW(camera_system_->SetSettings(setting));
//   brightness["valueB"] = 0;

//   // absValue is not double
//   brightness["absValue"] = 0;
//   properties.reset();
//   properties.push_back(brightness);
//   setting["property"] = properties;
//   ASSERT_ANY_THROW(camera_system_->SetSettings(setting));
//   brightness["absValue"] = 0.0;

//   // mode of trigger_mode is not int
//   trigger_mode["mode"] = 0.0;
//   properties.reset();
//   properties.push_back(trigger_mode);
//   setting["property"] = properties;
//   ASSERT_ANY_THROW(camera_system_->SetSettings(setting));
//   trigger_mode["mode"] = 0;

//   // polarity of trigger_mode is not int
//   trigger_mode["polarity"] = 0.0;
//   properties.reset();
//   properties.push_back(trigger_mode);
//   setting["property"] = properties;
//   ASSERT_ANY_THROW(camera_system_->SetSettings(setting));
//   trigger_mode["polarity"] = 0;
// }

}  // end of namespace tmc_pgr_camera

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
