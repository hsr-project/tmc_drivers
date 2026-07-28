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
/// @brief Retrieve the settings of the Point Grey camera system from the parameter server
///             and test whether they can be processed as appropriate values
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include "tmc_pgr_camera/ros_parameter_point_grey_camera_system_setting.hpp"
#include "tmc_pgr_camera/yaml_point_grey_camera_system_setting.hpp"

namespace {
// ROS parameter name
// Camera ID
const char* const kParamNameCamera = "camera";
// Property
const char* const kParamNamePropertyBrightnessOnOff = "property.brightness.on_off";
const char* const kParamNamePropertyAutoExposureOnOff = "property.auto_exposure.on_off";
const char* const kParamNamePropertySharpnessOnOff =  "property.sharpness.on_off";
const char* const kParamNamePropertyWhiteBalanceOnOff =  "property.white_balance.on_off";
const char* const kParamNamePropertyHueOnOff =  "property.hue.on_off";
const char* const kParamNamePropertySaturationOnOff =  "property.saturation.on_off";
const char* const kParamNamePropertyGammaOnOff =  "property.gamma.on_off";
const char* const kParamNamePropertyIrisOnOff =  "property.iris.on_off";
const char* const kParamNamePropertyFocusOnOff =  "property.focus.on_off";
const char* const kParamNamePropertyZoomOnOff =  "property.zoom.on_off";
const char* const kParamNamePropertyPanOnOff =  "property.pan.on_off";
const char* const kParamNamePropertyTiltOnOff =  "property.tilt.on_off";
const char* const kParamNamePropertyShutterOnOff =  "property.shutter.on_off";
const char* const kParamNamePropertyGainOnOff =  "property.gain.on_off";
const char* const kParamNamePropertyTriggerModeOnOff =  "property.trigger_mode.on_off";
const char* const kParamNamePropertyTriggerDelayOnOff =  "property.trigger_delay.on_off";
const char* const kParamNamePropertyFrameRateOnOff =  "property.frame_rate.on_off";
const char* const kParamNamePropertyTemperatureOnOff =  "property.temperature.on_off";
// Frame rate
const char* const kParamNameFrameRate = "frame_rate";
// Video mode
const char* const kParamNameVideoMode = "video_mode";
// format7 detailed settings
const char* const kParamNameFormat7Mode = "format7.mode";
const char* const kParamNameFormat7OffsetX = "format7.offset_x";
const char* const kParamNameFormat7OffsetY = "format7.offset_y";
const char* const kParamNameFormat7Width = "format7.width";
const char* const kParamNameFormat7Height = "format7.height";
const char* const kParamNameFormat7PixelFormat = "format7.pixel_format";
// Color demosaicing algorithm
const char* const kParamNameSoftwareDemosaicing = "software_demosaicing";
// Monocular/Stereo trigger
// No parameters
// Stereo trigger details
const char* const kParamNameSelfTriggerIo = "self_trigger.io";
const char* const kParamNameSelfTriggerPulseWidth = "self_trigger.pulse_width";
const char* const kParamNameSelfTriggerNumberOfPulse = "self_trigger.number_of_pulse";
const char* const kParamNameSelfTriggerPolarity = "self_trigger.polarity";
// Trigger mode
const char* const kParamNameTriggerModeMode = "trigger_mode.mode";
const char* const kParamNameTriggerModeOnOff = "trigger_mode.on_off";
const char* const kParamNameTriggerModePolarity = "trigger_mode.polarity";
// Trigger delay
// Included in property
// Gray -> RGB conversion
const char* const kParamNameChangeRgbFlag = "change_rgb_flag";
// Blackfly-specific 3.3V output settings
const char* const kParamNameOutputVoltage = "output_voltage";

// Parameter settings value
// Camera ID
const int kTestCameraId1 = 10000000;
const int kTestCameraId2 = 20000000;
const int kTestCameraId3 = 30000000;
const int kTestCameraId4 = -1;
// Property
const bool kTestProperty = true;
// Frame rate
const float kTestFrameRate1 = 7.5;
const float kTestFrameRate2 = 1.0;
const float kTestFrameRate3 = 1.875;
// Video mode
const char* const kTestVideoMode1 = "invalid_video_mode";
const char* const kTestVideoMode2 = "format7";
// format7 detailed settings
const int kTestFormat7Mode1 = -1;
const int kTestFormat7Mode2 = 32;
const int kTestFormat7Mode3 = 0;
const int kTestFormat7OffsetX = 8;
const int kTestFormat7OffsetY = 2;
const int kTestFormat7Width = 1280;
const int kTestFormat7Height = 960;
const char* const kTestFormat7PixelFormat1 = "invalid_pixel_format";
const char* const kTestFormat7PixelFormat2 = "raw8";
// Color demosaicing
const char* const kTestSoftwareDemosaicing1 = "invalid_demosaicing";
const char* const kTestSoftwareDemosaicing2 = "edge_sensing";
// Monocular/Stereo trigger
// No parameters
// Stereo trigger details
const int kTestSelfTriggerIo1 = 0;
const int kTestSelfTriggerIo2 = -1;
const int kTestSelfTriggerPulseWidth1 = 1;
const int kTestSelfTriggerPulseWidth2 = -1;
const int kTestSelfTriggerPulseWidth3 = 64;
const int kTestSelfTriggerPulseWidth4 = 63;
const uint32_t kExpectedSelfTriggerPulseWidth = 0xFC000400;
const int kTestSelfTriggerNumberOfPulse1 = 0x00;
const int kTestSelfTriggerNumberOfPulse2 = 0x100;
const int kTestSelfTriggerNumberOfPulse3 = 0xFF;
const int kTestSelfTriggerPolarity1 = -1;
const int kTestSelfTriggerPolarity2 = 2;
const int kTestSelfTriggerPolarity3 = 0;
const int kTestSelfTriggerPolarity4 = 1;
// Trigger mode
const int kTestTriggerModeMode1 = -1;
const int kTestTriggerModeMode2 = 15;
const int kTestTriggerModeMode3 = 14;
const bool kTestTriggerModeOnOff = true;
const int kTestTriggerModePolarity1 = -1;
const int kTestTriggerModePolarity2 = 2;
const int kTestTriggerModePolarity3 = 0;
const int kTestTriggerModePolarity4 = 1;
// Trigger delay
const bool kTestTriggerDelay = true;
// Gray -> RGB conversion
const bool kTestChangeRgbFlag = true;
// Blackfly-specific 3.3V output settings
const char* const kTestOutputVoltage1 = "true";
const int kTestOutputVoltage2 = 1;
const bool kTestOutputVoltage3 = true;
}  // anonymous namespace

namespace tmc_pgr_camera {
/// @brief Test fixture for loading Point Grey camera system settings from the parameter server
/// @note When the same parameters as the configuration file class are prepared on the parameter server,
///       verify whether they are correctly converted and the settings are applied as before
/// @note Additionally, perform tests for valid and invalid values
class RosParameterPointGreyCameraSystemSettingTest : public testing::Test {
 public:
  /// @brief Contains configuration file class and parameter server class
  virtual void SetUp() {
    private_node_handle_ = rclcpp::Node::make_shared("ros_param_setting_test");
    std::string config_path("test/config/default.yml");
    std::string camera_setting_file_path =
      private_node_handle_->declare_parameter("camera_setting_file_path", config_path);
    std::vector<int> camera_ids;
    EXPECT_NO_THROW(setting_file_.reset(new YamlPointGreyCameraSystemSetting(camera_setting_file_path)));
    const std::vector<uint32_t> camera_ids_uint = setting_file_->GetSerialNumbers();
    for (std::vector<uint32_t>::const_iterator it = camera_ids_uint.begin(); it != camera_ids_uint.end(); ++it) {
      camera_ids.push_back(static_cast<int>(*it));
    }
    // Register to the parameter server
    private_node_handle_->declare_parameter(kParamNameCamera, camera_ids);
    ASSERT_NO_THROW(ros_param_setting_.reset(new RosParameterPointGreyCameraSystemSetting(private_node_handle_)));
  }

  /// @brief Once a static typed parameter is declared, it cannot be undeclared, so declare separately
  void declare_parameters() {
    // Declare all parameters to be used in advance,
    // Undeclare unused parameters at the beginning of each TEST_F.
    private_node_handle_->declare_parameter(kParamNamePropertyBrightnessOnOff, true);
    private_node_handle_->declare_parameter("property.brightness.abs_value", 0.0);
    private_node_handle_->declare_parameter("property.brightness.one_push", true);
    private_node_handle_->declare_parameter("property.brightness.auto_manual_mode", false);
    private_node_handle_->declare_parameter(kParamNamePropertyAutoExposureOnOff, true);
    private_node_handle_->declare_parameter("property.auto_exposure.abs_value", 0.0);
    private_node_handle_->declare_parameter("property.auto_exposure.one_push", false);
    private_node_handle_->declare_parameter("property.auto_exposure.auto_manual_mode", false);
    private_node_handle_->declare_parameter(kParamNamePropertySharpnessOnOff, false);
    private_node_handle_->declare_parameter("property.sharpness.abs_value", 1024.0);
    private_node_handle_->declare_parameter("property.sharpness.one_push", true);
    private_node_handle_->declare_parameter("property.sharpness.auto_manual_mode", false);
    private_node_handle_->declare_parameter(kParamNamePropertyWhiteBalanceOnOff, true);
    private_node_handle_->declare_parameter("property.white_balance.value_a", 570);
    private_node_handle_->declare_parameter("property.white_balance.value_b", 810);
    private_node_handle_->declare_parameter("property.white_balance.one_push", false);
    private_node_handle_->declare_parameter("property.white_balance.auto_manual_mode", true);
    private_node_handle_->declare_parameter(kParamNamePropertyShutterOnOff, true);
    private_node_handle_->declare_parameter("property.shutter.abs_value", 20.0);
    private_node_handle_->declare_parameter("property.shutter.one_push", false);
    private_node_handle_->declare_parameter("property.shutter.auto_manual_mode", false);
    private_node_handle_->declare_parameter(kParamNamePropertyGainOnOff, true);
    private_node_handle_->declare_parameter("property.gain.abs_value", 0.0);
    private_node_handle_->declare_parameter("property.gain.one_push", false);
    private_node_handle_->declare_parameter("property.gain.auto_manual_mode", false);
    private_node_handle_->declare_parameter(kParamNamePropertyTriggerDelayOnOff, false);
    private_node_handle_->declare_parameter("property.trigger_delay.abs_value", 5.0);
    private_node_handle_->declare_parameter("property.trigger_delay.abs_control", true);

    private_node_handle_->declare_parameter(kParamNameFormat7Mode, 0);
    private_node_handle_->declare_parameter(kParamNameFormat7OffsetX, 8);
    private_node_handle_->declare_parameter(kParamNameFormat7OffsetY, 2);
    private_node_handle_->declare_parameter(kParamNameFormat7Width, 1280);
    private_node_handle_->declare_parameter(kParamNameFormat7Height, 960);
    private_node_handle_->declare_parameter(kParamNameFormat7PixelFormat, "raw8");

    private_node_handle_->declare_parameter(kParamNameSelfTriggerIo, std::vector<int64_t>{0, 1});
    private_node_handle_->declare_parameter(kParamNameSelfTriggerPulseWidth, std::vector<int64_t>{1, 16});
    private_node_handle_->declare_parameter(kParamNameSelfTriggerNumberOfPulse, 1);
    private_node_handle_->declare_parameter(kParamNameSelfTriggerPolarity, 0);

    private_node_handle_->declare_parameter(kParamNameSoftwareDemosaicing, kTestSoftwareDemosaicing2);
    private_node_handle_->declare_parameter(kParamNameVideoMode, kTestVideoMode2);
    private_node_handle_->declare_parameter(kParamNameFrameRate, kTestFrameRate1);

    private_node_handle_->declare_parameter(kParamNameTriggerModeMode, 0);
    private_node_handle_->declare_parameter(kParamNameTriggerModeOnOff, true);
    private_node_handle_->declare_parameter(kParamNameTriggerModePolarity, 0);

    private_node_handle_->declare_parameter(kParamNameChangeRgbFlag, kTestChangeRgbFlag);
    private_node_handle_->declare_parameter(kParamNameOutputVoltage, false);
  }

  /// @brief Set the minimum parameters required to instantiate the configuration class before exiting
  /// @note In ROS 2, parameters are specific to each node, so this is unnecessary
  virtual void TearDown() {
    // private_node_handle_->set_parameters({
    //   rclcpp::Parameter(kParamNameFrameRate, kTestFrameRate1),
    //   rclcpp::Parameter(kParamNameVideoMode, kTestVideoMode2),
    // });
  }
  /// @brief Returns a reference to the configuration file class
  inline std::shared_ptr<YamlPointGreyCameraSystemSetting> get_setting_file_ptr() const { return setting_file_; }
  /// @brief Returns a reference to the parameter server class
  inline std::shared_ptr<RosParameterPointGreyCameraSystemSetting>
      get_ros_param_setting_ptr() const { return ros_param_setting_; }
  /// @brief Returns the private node handle
  inline rclcpp::Node::SharedPtr get_private_node_handle() const { return private_node_handle_; }

 private:
  rclcpp::Node::SharedPtr private_node_handle_;
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting_file_;
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting_;
};

/// @brief Compare camera serials
TEST_F(RosParameterPointGreyCameraSystemSettingTest, CompareSerialNumbers) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting_file = get_setting_file_ptr();

  // Excercise
  std::vector<uint32_t> serial_ros = ros_param_setting->GetSerialNumbers();
  std::vector<uint32_t> serial_yaml = setting_file->GetSerialNumbers();

  // Verify
  ASSERT_EQ(serial_ros.size(), serial_yaml.size());
  std::vector<uint32_t>::iterator it_serial_ros = serial_ros.begin();
  std::vector<uint32_t>::iterator it_serial_yaml = serial_yaml.begin();
  for (; it_serial_ros != serial_ros.end(); ++it_serial_ros, ++it_serial_yaml) {
    EXPECT_EQ(*it_serial_ros, *it_serial_yaml);
  }
}

/// @brief Compare properties
TEST_F(RosParameterPointGreyCameraSystemSettingTest, CompareProperties) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting_file = get_setting_file_ptr();
  declare_parameters();

  // Excercise
  std::vector<FlyCapture2::Property> properties_ros = ros_param_setting->GetProperties();
  std::vector<FlyCapture2::Property> properties_yaml = setting_file->GetProperties();

  // Verify
  for (std::vector<FlyCapture2::Property>::iterator it_properties_ros = properties_ros.begin();
       it_properties_ros != properties_ros.end(); ++it_properties_ros) {
    for (std::vector<FlyCapture2::Property>::iterator it_properties_yaml = properties_yaml.begin();
         it_properties_yaml != properties_yaml.end(); ++it_properties_yaml) {
      if (it_properties_ros->type == it_properties_yaml->type) {
        EXPECT_EQ(it_properties_ros->onOff, it_properties_yaml->onOff) << "At " << it_properties_ros->type;
        EXPECT_EQ(it_properties_ros->absControl, it_properties_yaml->absControl) << "At " << it_properties_ros->type;
        EXPECT_FLOAT_EQ(it_properties_ros->absValue, it_properties_yaml->absValue) << "At " << it_properties_ros->type;
        EXPECT_EQ(it_properties_ros->onePush, it_properties_yaml->onePush) << "At " << it_properties_ros->type;
        EXPECT_EQ(it_properties_ros->present, it_properties_yaml->present) << "At " << it_properties_ros->type;
        EXPECT_EQ(it_properties_ros->autoManualMode, it_properties_yaml->autoManualMode)
            << "At " << it_properties_ros->type;
        EXPECT_EQ(it_properties_ros->valueA, it_properties_yaml->valueA) << "At " << it_properties_ros->type;
        EXPECT_EQ(it_properties_ros->valueB, it_properties_yaml->valueB) << "At " << it_properties_ros->type;
      }
    }
  }
}

/// @brief Compare video modes and frame rates
TEST_F(RosParameterPointGreyCameraSystemSettingTest, CompareVideoModeAndFrameRate) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting_file = get_setting_file_ptr();
  declare_parameters();

  // Excercise, Verify
  std::optional<FlyCapture2::VideoMode> video_mode_ros = ros_param_setting->GetVideoMode();
  std::optional<FlyCapture2::VideoMode> video_mode_yaml = setting_file->GetVideoMode();
  ASSERT_EQ(*video_mode_ros, *video_mode_yaml);
  std::optional<std::pair<FlyCapture2::FrameRate, float> > frame_rate_ros =
      ros_param_setting->GetFrameRate(*video_mode_ros);
  std::optional<std::pair<FlyCapture2::FrameRate, float> > frame_rate_yaml =
      setting_file->GetFrameRate(*video_mode_yaml);
  EXPECT_EQ(frame_rate_ros->first, frame_rate_yaml->first);
  ASSERT_FLOAT_EQ(frame_rate_ros->second, frame_rate_yaml->second);
}

/// @brief Compare format7 settings
TEST_F(RosParameterPointGreyCameraSystemSettingTest, CompareFormat7) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting_file = get_setting_file_ptr();
  declare_parameters();

  // Excercise
  std::optional<FlyCapture2::Format7ImageSettings> format7_ros = ros_param_setting->GetFormat7Setting();
  std::optional<FlyCapture2::Format7ImageSettings> format7_yaml = setting_file->GetFormat7Setting();

  // Verify
  EXPECT_EQ(format7_ros->mode, format7_yaml->mode);
  EXPECT_EQ(format7_ros->offsetX, format7_yaml->offsetX);
  EXPECT_EQ(format7_ros->offsetY, format7_yaml->offsetY);
  EXPECT_EQ(format7_ros->width, format7_yaml->width);
  EXPECT_EQ(format7_ros->height, format7_yaml->height);
  EXPECT_EQ(format7_ros->pixelFormat, format7_yaml->pixelFormat);
}

/// @brief Compare color demosaicing settings
TEST_F(RosParameterPointGreyCameraSystemSettingTest, CompareSoftDemosaicing) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting_file = get_setting_file_ptr();
  declare_parameters();

  // Excercise
  std::optional<FlyCapture2::ColorProcessingAlgorithm> soft_demosaicing_ros = ros_param_setting->GetSoftDemosaicing();
  std::optional<FlyCapture2::ColorProcessingAlgorithm> soft_demosaicing_yaml = setting_file->GetSoftDemosaicing();

  // Verify
  ASSERT_EQ(*soft_demosaicing_ros, *soft_demosaicing_yaml);
}

/// @brief Compare software trigger enable settings
TEST_F(RosParameterPointGreyCameraSystemSettingTest, CompareSoftwareTrigger) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting_file = get_setting_file_ptr();
  declare_parameters();

  // Excercise
  bool software_trigger_enable_ros = ros_param_setting->IsSoftwareTriggerEnabled();
  bool software_trigger_enable_yaml = setting_file->IsSoftwareTriggerEnabled();

  // Verify
  ASSERT_EQ(software_trigger_enable_ros, software_trigger_enable_yaml);
}

/// @brief Compare self-trigger enable settings
TEST_F(RosParameterPointGreyCameraSystemSettingTest, CompareSelfTrigger) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting_file = get_setting_file_ptr();
  declare_parameters();

  // Excercise
  bool self_trigger_enable_ros = ros_param_setting->IsSelfTriggerEnabled();
  bool self_trigger_enable_yaml = setting_file->IsSelfTriggerEnabled();

  // Verify
  ASSERT_EQ(self_trigger_enable_ros, self_trigger_enable_yaml);
}

/// @brief Compare self-trigger properties
TEST_F(RosParameterPointGreyCameraSystemSettingTest, CompareSelfTriggerSettings) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting_file = get_setting_file_ptr();
  declare_parameters();

  // Excercise
  std::optional<SelfTriggerSettings> self_trigger_setting_ros = ros_param_setting->GetSelfTriggerSettings();
  std::optional<SelfTriggerSettings> self_trigger_setting_yaml = setting_file->GetSelfTriggerSettings();

  // Veriry
  EXPECT_EQ(self_trigger_setting_ros->in_io, self_trigger_setting_yaml->in_io);
  EXPECT_EQ(self_trigger_setting_ros->out_io, self_trigger_setting_yaml->out_io);
  EXPECT_EQ(self_trigger_setting_ros->pulse_figure, self_trigger_setting_yaml->pulse_figure);
  EXPECT_EQ(self_trigger_setting_ros->number_of_pulse, self_trigger_setting_yaml->number_of_pulse);
  EXPECT_EQ(self_trigger_setting_ros->polarity, self_trigger_setting_yaml->polarity);
}

/// @brief Compare trigger modes
TEST_F(RosParameterPointGreyCameraSystemSettingTest, CompareTriggerMode) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting_file = get_setting_file_ptr();
  declare_parameters();

  // Excercise
  std::optional<FlyCapture2::TriggerMode> trigger_mode_ros = ros_param_setting->GetTriggerMode();
  std::optional<FlyCapture2::TriggerMode> trigger_mode_yaml = setting_file->GetTriggerMode();

  // Veriry
  EXPECT_EQ(trigger_mode_ros->mode, trigger_mode_yaml->mode);
  EXPECT_EQ(trigger_mode_ros->onOff, trigger_mode_yaml->onOff);
  EXPECT_EQ(trigger_mode_ros->parameter, trigger_mode_yaml->parameter);
  EXPECT_EQ(trigger_mode_ros->source, trigger_mode_yaml->source);
}

/// @brief Compare trigger delays
TEST_F(RosParameterPointGreyCameraSystemSettingTest, CompareTriggerDelay) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting_file = get_setting_file_ptr();
  declare_parameters();

  // Excercise
  std::optional<FlyCapture2::TriggerDelay> trigger_delay_ros = ros_param_setting->GetTriggerDelay();
  std::optional<FlyCapture2::TriggerDelay> trigger_delay_yaml = setting_file->GetTriggerDelay();

  // Verify
  EXPECT_EQ(trigger_delay_ros->onOff, trigger_delay_yaml->onOff);
  EXPECT_EQ(trigger_delay_ros->absControl, trigger_delay_yaml->absControl);
  EXPECT_FLOAT_EQ(trigger_delay_ros->absValue, trigger_delay_yaml->absValue);
  EXPECT_EQ(trigger_delay_ros->onePush, trigger_delay_yaml->onePush);
  EXPECT_EQ(trigger_delay_ros->present, trigger_delay_yaml->present);
  EXPECT_EQ(trigger_delay_ros->autoManualMode, trigger_delay_yaml->autoManualMode);
  EXPECT_EQ(trigger_delay_ros->valueA, trigger_delay_yaml->valueA);
  EXPECT_EQ(trigger_delay_ros->valueB, trigger_delay_yaml->valueB);
}

/// @brief Compare output image formats
TEST_F(RosParameterPointGreyCameraSystemSettingTest, CompareImageType) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting_file = get_setting_file_ptr();
  declare_parameters();

  // Excercise
  std::optional<ImageType> image_type_ros = ros_param_setting->GetImageType();
  std::optional<ImageType> image_type_yaml = setting_file->GetImageType();

  // Verify
  ASSERT_EQ(*image_type_ros, *image_type_yaml);
}

/// @brief Compare 3.3V output settings
TEST_F(RosParameterPointGreyCameraSystemSettingTest, CompareOutputVoltage) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting_file = get_setting_file_ptr();
  declare_parameters();

  // Excercise
  std::optional<bool> output_voltage_ros = ros_param_setting->GetOutputVoltageSetting();
  std::optional<bool> output_voltage_yaml = setting_file->GetOutputVoltageSetting();

  // Verify
  ASSERT_EQ(*output_voltage_ros, *output_voltage_yaml);
}

/// @brief Camera serial (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSerialEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  // Incorrect(Empty)
  std::vector<int> camera_ids;
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameCamera, camera_ids));
  ASSERT_ANY_THROW(ros_param_setting->GetSerialNumbers());
}

/// @brief Camera serial (invalid length)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSerialInvalidLength) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  // Incorrect(Invalid length)
  std::vector<int> camera_ids;
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameCamera, camera_ids));
  EXPECT_ANY_THROW(ros_param_setting->GetSerialNumbers());
  camera_ids.push_back(kTestCameraId1);
  camera_ids.push_back(kTestCameraId2);
  camera_ids.push_back(kTestCameraId3);
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameCamera, camera_ids));
  ASSERT_ANY_THROW(ros_param_setting->GetSerialNumbers());
}

/// @brief Camera serial (invalid value)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSerialInvalidValue) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  // Incorrect(Invalid value)
  std::vector<int> camera_ids;
  camera_ids.push_back(kTestCameraId1);
  camera_ids.push_back(kTestCameraId4);
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameCamera, camera_ids));
  ASSERT_ANY_THROW(ros_param_setting->GetSerialNumbers());
}

/// @brief Camera serial (valid value)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSerialValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  // Correct
  std::vector<int> camera_ids;
  camera_ids.push_back(kTestCameraId1);
  camera_ids.push_back(kTestCameraId2);
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameCamera, camera_ids));
  std::vector<uint32_t> serials;
  ASSERT_NO_THROW(serials = ros_param_setting->GetSerialNumbers());
  EXPECT_EQ(serials.at(0), kTestCameraId1);
  ASSERT_EQ(serials.at(1), kTestCameraId2);
}

/// @brief Property valid and invalid values
/// @note The expected values are not fixed, so check whether they can be retrieved or not
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfProperties) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  // Incorrect(Empty)
  ASSERT_ANY_THROW(ros_param_setting->GetProperties());
  // Correct
  get_private_node_handle()->declare_parameter(kParamNamePropertyBrightnessOnOff,  kTestProperty);
  std::vector<FlyCapture2::Property> properties;
  ASSERT_NO_THROW(properties = ros_param_setting->GetProperties());
  ASSERT_EQ(properties.at(0).type, FlyCapture2::BRIGHTNESS);
}

/// @brief Frame rate (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFrameRateEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  get_private_node_handle()->declare_parameter(kParamNameVideoMode, kTestVideoMode2);

  // Excercise and verify
  std::optional<FlyCapture2::VideoMode> video_mode;
  ASSERT_NO_THROW(video_mode = ros_param_setting->GetVideoMode());
  ASSERT_ANY_THROW(ros_param_setting->GetFrameRate(*video_mode));
}

/// @brief Frame rate (valid case 1)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFrameRateValidFormat7) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameFrameRate, kTestFrameRate1));
  std::optional<std::pair<FlyCapture2::FrameRate, float> > frame_rate;

  // Excercise and verify
  // In case of other video_mode.
  // Correct
  std::optional<FlyCapture2::VideoMode> video_mode;
  ASSERT_NO_THROW(video_mode = ros_param_setting->GetVideoMode());
  ASSERT_NO_THROW(frame_rate = ros_param_setting->GetFrameRate(*video_mode));
  EXPECT_EQ(frame_rate->first, FlyCapture2::FRAMERATE_FORMAT7);
  ASSERT_NEAR(frame_rate->second, kTestFrameRate1, std::numeric_limits<float>::epsilon());
}

/// @brief Frame rate (invalid combination with video mode)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFrameRateInvalidVideoMode) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameFrameRate, kTestFrameRate2));

  // Excercise and verify
  // In case of other video_mode.
  // (Accepts only specified frame rate.)
  // Incorrect(Invalid)
  ASSERT_ANY_THROW(ros_param_setting->GetFrameRate(FlyCapture2::VIDEOMODE_1280x960RGB));
}

/// @brief Frame rate (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFrameRateValidFrameRate) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameFrameRate, kTestFrameRate3));
  std::optional<std::pair<FlyCapture2::FrameRate, float> > frame_rate;

  // Excercise and verify
  // Correct
  ASSERT_NO_THROW(frame_rate = ros_param_setting->GetFrameRate(FlyCapture2::VIDEOMODE_1280x960RGB));
  EXPECT_EQ(frame_rate->first, FlyCapture2::FRAMERATE_1_875);
  ASSERT_NEAR(frame_rate->second, kTestFrameRate3, std::numeric_limits<float>::epsilon());
}

/// @brief Video mode (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfVideoModeEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetVideoMode());
}

/// @brief Video mode (invalid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfVideoModeInvalid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameVideoMode, kTestVideoMode1));

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetVideoMode());
}

/// @brief Video mode (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfVideoModeValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameVideoMode, kTestVideoMode2));
  std::optional<FlyCapture2::VideoMode> video_mode;

  // Excercise and verify
  ASSERT_NO_THROW(video_mode = ros_param_setting->GetVideoMode());
  ASSERT_EQ(video_mode, FlyCapture2::VIDEOMODE_FORMAT7);
}

/// @brief Format7 mode (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFormat7SettingModeEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetFormat7Setting());
}

/// @brief Format7 mode (invalid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFormat7SettingModeInvalid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();

  // Excercise and verify
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameFormat7Mode, kTestFormat7Mode1));
  EXPECT_ANY_THROW(ros_param_setting->GetFormat7Setting());
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameFormat7Mode, kTestFormat7Mode2));
  ASSERT_ANY_THROW(ros_param_setting->GetFormat7Setting());
}

/// @brief Format7 mode (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFormat7SettingModeValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();

  // Excercise and verify
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameFormat7Mode, kTestFormat7Mode3));
  std::optional<FlyCapture2::Format7ImageSettings> format7_setting;
  ASSERT_NO_THROW(format7_setting = ros_param_setting->GetFormat7Setting());
  ASSERT_EQ(format7_setting->mode, kTestFormat7Mode3);
}

/// @brief Format7 offset X (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFormat7SettingOffsetXEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetFormat7Setting());
}

/// @brief Format7 offset X (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFormat7SettingOffsetXValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::optional<FlyCapture2::Format7ImageSettings> format7_setting;
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameFormat7OffsetX, kTestFormat7OffsetX));

  // Excercise and verify
  ASSERT_NO_THROW(format7_setting = ros_param_setting->GetFormat7Setting());
  ASSERT_EQ(format7_setting->offsetX, kTestFormat7OffsetX);
}

/// @brief Format7 offset Y (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFormat7SettingOffsetYEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetFormat7Setting());
}

/// @brief Format7 offset Y (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFormat7SettingOffsetYValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::optional<FlyCapture2::Format7ImageSettings> format7_setting;
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameFormat7OffsetY, kTestFormat7OffsetY));

  // Excercise and verify
  ASSERT_NO_THROW(format7_setting = ros_param_setting->GetFormat7Setting());
  ASSERT_EQ(format7_setting->offsetY, kTestFormat7OffsetY);
}

/// @brief Format7 width (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFormat7SettingWidthEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetFormat7Setting());
}

/// @brief Format7 width (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFormat7SettingWidthValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::optional<FlyCapture2::Format7ImageSettings> format7_setting;
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameFormat7Width, kTestFormat7Width));

  // Excercise and verify
  ASSERT_NO_THROW(format7_setting = ros_param_setting->GetFormat7Setting());
  ASSERT_EQ(format7_setting->width, kTestFormat7Width);
}

/// @brief Format7 height (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFormat7SettingHeightEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetFormat7Setting());
}

/// @brief Format7 height (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFormat7SettingHeightValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::optional<FlyCapture2::Format7ImageSettings> format7_setting;
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameFormat7Height, kTestFormat7Height));

  // Excercise and verify
  ASSERT_NO_THROW(format7_setting = ros_param_setting->GetFormat7Setting());
  ASSERT_EQ(format7_setting->height, kTestFormat7Height);
}

/// @brief Format7 pixel format (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFormat7SettingPixelFormatEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetFormat7Setting());
}

/// @brief Format7 pixel format (invalid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFormat7SettingPixelFormatInvalid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameFormat7PixelFormat, kTestFormat7PixelFormat1));

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetFormat7Setting());
}

/// @brief Format7 pixel format (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfFormat7SettingPixelFormatValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::optional<FlyCapture2::Format7ImageSettings> format7_setting;
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameFormat7PixelFormat, kTestFormat7PixelFormat2));

  // Excercise and verify
  ASSERT_NO_THROW(format7_setting = ros_param_setting->GetFormat7Setting());
  ASSERT_EQ(format7_setting->pixelFormat, FlyCapture2::PIXEL_FORMAT_RAW8);
}

/// @brief Demosaicing (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSoftDemosaicingEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetSoftDemosaicing());
}

/// @brief Demosaicing (invalid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSoftDemosaicingInvalid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameSoftwareDemosaicing, kTestSoftwareDemosaicing1));

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetSoftDemosaicing());
}

/// @brief Demosaicing (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSoftDemosaicingValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameSoftwareDemosaicing, kTestSoftwareDemosaicing2));

  // Excercise and verify
  std::optional<FlyCapture2::ColorProcessingAlgorithm> soft_demosaicing;
  ASSERT_NO_THROW(soft_demosaicing = ros_param_setting->GetSoftDemosaicing());
  ASSERT_EQ(soft_demosaicing, FlyCapture2::EDGE_SENSING);
}

/// @brief Monocular and stereo shutter settings (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfShutterEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  // NOTE: In ROS 2, undeclare is not possible, so set to empty
  std::vector<int> camera_ids;
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameCamera, camera_ids));

  // Excercise and verify
  EXPECT_ANY_THROW(ros_param_setting->IsSoftwareTriggerEnabled());
  ASSERT_ANY_THROW(ros_param_setting->IsSelfTriggerEnabled());
}

/// @brief Monocular shutter settings (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfShutterMonoValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::vector<int> monocular;
  monocular.push_back(kTestCameraId1);

  // Excercise and verify
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameCamera, monocular));
  EXPECT_TRUE(ros_param_setting->IsSoftwareTriggerEnabled());
  EXPECT_FALSE(ros_param_setting->IsSelfTriggerEnabled());
}

/// @brief Stereo shutter settings (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfShutterStereoValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::vector<int> stereo;
  stereo.push_back(kTestCameraId1);
  stereo.push_back(kTestCameraId2);

  // Excercise and verify
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameCamera, stereo));
  EXPECT_FALSE(ros_param_setting->IsSoftwareTriggerEnabled());
  EXPECT_TRUE(ros_param_setting->IsSelfTriggerEnabled());
}

/// @brief Stereo shutter detailed settings IO (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSelfTriggerSettingIOEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetSelfTriggerSettings());
}

/// @brief Stereo shutter detailed settings IO (invalid length)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSelfTriggerSettingIOInvalidLength) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::vector<int> io;

  // Excercise and verify
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameSelfTriggerIo, io));
  EXPECT_ANY_THROW(ros_param_setting->GetSelfTriggerSettings());
  io.push_back(kTestSelfTriggerIo1);
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameSelfTriggerIo, io));
  ASSERT_ANY_THROW(ros_param_setting->GetSelfTriggerSettings());
}

/// @brief Stereo shutter detailed settings IO (invalid value)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSelfTriggerSettingIOInvalidValue) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::vector<int> io;
  io.push_back(kTestSelfTriggerIo1);
  io.push_back(kTestSelfTriggerIo2);
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameSelfTriggerIo, io));

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetSelfTriggerSettings());
}

/// @brief Stereo shutter detailed settings IO (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSelfTriggerSettingIOValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::vector<int> io;
  io.push_back(kTestSelfTriggerIo1);
  io.push_back(kTestSelfTriggerIo1);
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameSelfTriggerIo, io));
  std::optional<SelfTriggerSettings> selftrigger_setting;

  // Excercise and verify
  ASSERT_NO_THROW(selftrigger_setting = ros_param_setting->GetSelfTriggerSettings());
  EXPECT_EQ(selftrigger_setting->in_io, kTestSelfTriggerIo1);
  ASSERT_EQ(selftrigger_setting->out_io, kTestSelfTriggerIo1);
}

/// @brief Stereo shutter detailed settings pulse width (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSelfTriggerSettingPulseWidthEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetSelfTriggerSettings());
}

/// @brief Stereo shutter detailed settings pulse width (invalid length)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSelfTriggerSettingPulseWidthInvalidLength) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::vector<int> pulse_width;

  // Excercise and verify
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameSelfTriggerPulseWidth, pulse_width));
  EXPECT_ANY_THROW(ros_param_setting->GetSelfTriggerSettings());
  pulse_width.push_back(kTestSelfTriggerPulseWidth1);
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameSelfTriggerPulseWidth, pulse_width));
  ASSERT_ANY_THROW(ros_param_setting->GetSelfTriggerSettings());
}

/// @brief Stereo shutter detailed settings pulse width (invalid value)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSelfTriggerSettingPulseWidthInvalidValue) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::vector<int> pulse_width;
  pulse_width.push_back(kTestSelfTriggerPulseWidth1);
  pulse_width.push_back(kTestSelfTriggerPulseWidth2);

  // Excercise and verify
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameSelfTriggerPulseWidth, pulse_width));
  EXPECT_ANY_THROW(ros_param_setting->GetSelfTriggerSettings());
  pulse_width.at(1) = kTestSelfTriggerPulseWidth3;
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameSelfTriggerPulseWidth, pulse_width));
  ASSERT_ANY_THROW(ros_param_setting->GetSelfTriggerSettings());
}

/// @brief Stereo shutter detailed settings pulse width (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSelfTriggerSettingPulseWidthValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::optional<SelfTriggerSettings> selftrigger_setting;
  std::vector<int> pulse_width;
  pulse_width.push_back(kTestSelfTriggerPulseWidth1);
  pulse_width.push_back(kTestSelfTriggerPulseWidth4);

  // Excercise and verify
  get_private_node_handle()->set_parameter(rclcpp::Parameter(kParamNameSelfTriggerPulseWidth, pulse_width));
  ASSERT_NO_THROW(selftrigger_setting = ros_param_setting->GetSelfTriggerSettings());
  ASSERT_EQ(selftrigger_setting->pulse_figure, kExpectedSelfTriggerPulseWidth);
}

/// @brief Stereo shutter detailed settings pulse count (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSelfTriggerSettingPulseNumEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetSelfTriggerSettings());
}

/// @brief Stereo shutter detailed settings pulse count (invalid value)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSelfTriggerSettingPulseNumInvalidValue) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();

  // Excercise and verify
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameSelfTriggerNumberOfPulse, kTestSelfTriggerNumberOfPulse1));
  EXPECT_ANY_THROW(ros_param_setting->GetSelfTriggerSettings());
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameSelfTriggerNumberOfPulse, kTestSelfTriggerNumberOfPulse2));
  ASSERT_ANY_THROW(ros_param_setting->GetSelfTriggerSettings());
}

/// @brief Stereo shutter detailed settings pulse count (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSelfTriggerSettingPulseNumValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::optional<SelfTriggerSettings> selftrigger_setting;

  // Excercise and verify
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameSelfTriggerNumberOfPulse, kTestSelfTriggerNumberOfPulse3));
  ASSERT_NO_THROW(selftrigger_setting = ros_param_setting->GetSelfTriggerSettings());
  ASSERT_EQ(selftrigger_setting->number_of_pulse, kTestSelfTriggerNumberOfPulse3);
}

/// @brief Stereo shutter detailed settings pulse polarity (negative value)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSelfTriggerSettingPolarityInvalidMinusValue) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();

  // Excercise and verify
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameSelfTriggerPolarity, kTestSelfTriggerPolarity1));
  ASSERT_ANY_THROW(ros_param_setting->GetSelfTriggerSettings());
}

/// @brief Stereo shutter detailed settings pulse polarity (out-of-range positive value)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSelfTriggerSettingPolarityInvalidLargeValue) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();

  // Excercise and verify
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameSelfTriggerPolarity, kTestSelfTriggerPolarity2));
  ASSERT_ANY_THROW(ros_param_setting->GetSelfTriggerSettings());
}

/// @brief Stereo shutter detailed settings pulse polarity (valid 1)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSelfTriggerSettingPolarityValid1) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::optional<SelfTriggerSettings> selftrigger_setting;

  // Excercise and verify
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameSelfTriggerPolarity, kTestSelfTriggerPolarity3));
  ASSERT_NO_THROW(selftrigger_setting = ros_param_setting->GetSelfTriggerSettings());
  ASSERT_EQ(selftrigger_setting->polarity, kTestSelfTriggerPolarity3);
}

/// @brief Stereo shutter detailed settings pulse polarity (valid 2)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfSelfTriggerSettingPolarityValid2) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::optional<SelfTriggerSettings> selftrigger_setting;

  // Excercise and verify
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameSelfTriggerPolarity, kTestSelfTriggerPolarity4));
  ASSERT_NO_THROW(selftrigger_setting = ros_param_setting->GetSelfTriggerSettings());
  ASSERT_EQ(selftrigger_setting->polarity, kTestSelfTriggerPolarity4);
}

/// @brief Trigger mode setting mode (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfTriggerModeModeEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetTriggerMode());
}

/// @brief Trigger mode setting mode (invalid value)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfTriggerModeModeInvalidValue) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();

  // Excercise and verify
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameTriggerModeMode, kTestTriggerModeMode1));
  EXPECT_ANY_THROW(ros_param_setting->GetTriggerMode());
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameTriggerModeMode, kTestTriggerModeMode2));
  ASSERT_ANY_THROW(ros_param_setting->GetTriggerMode());
}

/// @brief Trigger mode setting mode (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfTriggerModeModeValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::optional<FlyCapture2::TriggerMode> trigger_mode;

  // Excercise and verify
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameTriggerModeMode, kTestTriggerModeMode3));
  ASSERT_NO_THROW(trigger_mode = ros_param_setting->GetTriggerMode());
  ASSERT_EQ(trigger_mode->mode, kTestTriggerModeMode3);
}

/// @brief Trigger mode setting On/Off (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfTriggerModeOnOffEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetTriggerMode());
}

/// @brief Trigger mode setting On/Off (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfTriggerModeOnOffValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::optional<FlyCapture2::TriggerMode> trigger_mode;

  // Excercise and verify
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameTriggerModeOnOff, kTestTriggerModeOnOff));
  ASSERT_NO_THROW(trigger_mode = ros_param_setting->GetTriggerMode());
  ASSERT_TRUE(trigger_mode->onOff);
}

/// @brief Trigger mode setting polarity (negative value)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfTriggerModePolarityInvalidMinus) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();

  // Excercise and verify
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameTriggerModePolarity, kTestTriggerModePolarity1));
  ASSERT_ANY_THROW(ros_param_setting->GetTriggerMode());
}

/// @brief Trigger mode setting polarity (invalid positive value)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfTriggerModePolarityInvalidLargeValue) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();

  // Excercise and verify
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameTriggerModePolarity, kTestTriggerModePolarity2));
  ASSERT_ANY_THROW(ros_param_setting->GetTriggerMode());
}

/// @brief Trigger mode setting polarity (valid 1)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfTriggerModePolarityValid1) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::optional<FlyCapture2::TriggerMode> trigger_mode;

  // Excercise and verify
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameTriggerModePolarity, kTestTriggerModePolarity3));
  ASSERT_NO_THROW(trigger_mode = ros_param_setting->GetTriggerMode());
  ASSERT_EQ(trigger_mode->polarity, kTestTriggerModePolarity3);
}

/// @brief Trigger mode setting polarity (valid 2)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfTriggerModePolarityValid2) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::optional<FlyCapture2::TriggerMode> trigger_mode;

  // Excercise and verify
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameTriggerModePolarity, kTestTriggerModePolarity4));
  ASSERT_NO_THROW(trigger_mode = ros_param_setting->GetTriggerMode());
  ASSERT_EQ(trigger_mode->polarity, kTestTriggerModePolarity4);
}

/// @brief Trigger delay setting (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfTriggerDelayEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetTriggerDelay());
}

/// @brief Trigger delay setting (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfTriggerDelayValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  std::optional<FlyCapture2::TriggerDelay> trigger_delay;
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNamePropertyTriggerDelayOnOff, kTestTriggerDelay));

  // Excercise and verify
  ASSERT_NO_THROW(trigger_delay = ros_param_setting->GetTriggerDelay());
  ASSERT_TRUE(trigger_delay->onOff);
}

/// @brief Gray->RGB conversion enable setting (empty)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfImageTypeEmpty) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();

  // Excercise and verify
  ASSERT_ANY_THROW(ros_param_setting->GetImageType());
}

/// @brief Gray->RGB conversion enable setting (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfImageTypeValid) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameChangeRgbFlag, kTestChangeRgbFlag));
  std::optional<ImageType> image_type;

  // Excercise and verify
  ASSERT_NO_THROW(image_type = ros_param_setting->GetImageType());
  ASSERT_EQ(image_type, kRgbImage);
}

/// @brief 3.3V output setting (invalid 1)
/// @note In ROS 2, assigning a different type to a statically typed parameter is not possible
// TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfOutputVoltageInvalid1) {
//   // Setup
//   std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
//   declare_parameters();
//   get_private_node_handle()->set_parameter(
//     rclcpp::Parameter(kParamNameOutputVoltage, kTestOutputVoltage1));

//   // Excercise and verify
//   ASSERT_ANY_THROW(ros_param_setting->GetOutputVoltageSetting());
// }

/// @brief 3.3V output setting (invalid 2)
/// @note In ROS 2, assigning a different type to a statically typed parameter is not possible
// TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfOutputVoltageInvalid2) {
//   // Setup
//   std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
//   declare_parameters();
//   get_private_node_handle()->set_parameter(
//     rclcpp::Parameter(kParamNameOutputVoltage, kTestOutputVoltage2));

//   // Excercise and verify
//   ASSERT_ANY_THROW(ros_param_setting->GetOutputVoltageSetting());
// }

/// @brief 3.3V output setting (valid)
TEST_F(RosParameterPointGreyCameraSystemSettingTest, ValueCheckingOfOutputVoltageInvalid3) {
  // Setup
  std::shared_ptr<RosParameterPointGreyCameraSystemSetting> ros_param_setting = get_ros_param_setting_ptr();
  declare_parameters();
  get_private_node_handle()->set_parameter(
    rclcpp::Parameter(kParamNameOutputVoltage, kTestOutputVoltage3));
  std::optional<bool> output_voltage;

  // Excercise and verify
  ASSERT_NO_THROW(output_voltage = ros_param_setting->GetOutputVoltageSetting());
  ASSERT_TRUE(*output_voltage);
}

}  // namespace tmc_pgr_camera

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
