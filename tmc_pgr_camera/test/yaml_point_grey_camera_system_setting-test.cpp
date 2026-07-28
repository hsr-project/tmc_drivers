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
/// @brief      Test for loading configuration files of the Point Grey camera system
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <boost/format.hpp>
#include <gtest/gtest.h>

#include "tmc_pgr_camera/yaml_point_grey_camera_system_setting.hpp"

namespace {

// Default file used for loading configuration files
const char* kDefaultConfigFileName = "test/config/default.yml";

// Camera serial number in the default configuration file
const int32_t kDefaultSerialNumber0 = 13442704;
const int32_t kDefaultSerialNumber1 = 13442617;

// Name of a non-existent file
const char* kInvalidFilePath = "none";

// File name used for serial number retrieval test
const char* kGetSerialNumbersNormalTestFileName = "test/config/get_serial_numbers_normal.yml";

// File name used for serial number retrieval error test
const char* kGetSerialNumbersAbnormalTestFileName = "test/config/get_serial_numbers_abnormal_%02d.yml";

// Number of files used for serial number retrieval error test
const int32_t kGetSerialNumbersAbnormalTestNumberOfFiles = 5;

// File name used for camera property retrieval error test
const char* kGetPropertiesAbnormalTestFileName = "test/config/get_properties_abnormal_%02d.yml";

// Number of files used for camera property retrieval error test
const int32_t kGetPropertiesAbnormalTestNumberOfFiles = 2;

// File name used for frame rate retrieval error test
const char* kGetFrameRateAbnormalTestFileName = "test/config/get_frame_rate_abnormal_%02d.yml";

// Number of files used for frame rate retrieval error test
const int32_t kGetFrameRateAbnormalTestNumberOfFiles = 2;

// File name used for video mode retrieval error test
const char* kGetVideoModeAbnormalTestFileName = "test/config/get_video_mode_abnormal_%02d.yml";

// Number of files used for video mode retrieval error test
const int32_t kGetVideoModeAbnormalTestNumberOfFiles = 2;

// File name used for Format7 settings retrieval error test
const char* kGetFormat7SettingAbnormalTestFileName = "test/config/get_format7_setting_abnormal_%02d.yml";

// Number of files used for Format7 settings retrieval error test
const int32_t kGetFormat7SettingAbnormalTestNumberOfFiles = 15;

// File name used for software demosaicing retrieval error test
const char* kGetSoftDemosaicingAbnormalTestFileName = "test/config/get_soft_demosaicing_abnormal_%02d.yml";

// Number of files used for software demosaicing retrieval error test
const int32_t kGetSoftDemosaicingAbnormalTestNumberOfFiles = 2;

// File name used for software trigger retrieval error test
const char* kIsSoftwareTriggerEnabledAbnormalTestFileName = "test/config/is_software_trigger_enabled_abnormal_%02d.yml";

// Number of files used for software trigger retrieval error test
const int32_t kIsSoftwareTriggerEnabledAbnormalTestNumberOfFiles = 1;

// File name used for self-trigger retrieval error test
const char* kIsSelfTriggerEnabledAbnormalTestFileName = "test/config/is_self_trigger_enabled_abnormal_%02d.yml";

// Number of files used for self-trigger retrieval error test
const int32_t kIsSelfTriggerEnabledAbnormalTestNumberOfFiles = 1;

// File name used for self-trigger settings retrieval error test
const char* kGetSelfTriggerSettingsAbnormalTestFileName = "test/config/get_self_trigger_settings_abnormal_%02d.yml";

// Number of files used for self-trigger settings retrieval error test
const int32_t kGetSelfTriggerSettingsAbnormalTestNumberOfFiles = 8;

// File name used for trigger mode settings retrieval test
const char* kGetTriggerModeNormalTestFileName = "test/config/get_trigger_mode_normal_%02d.yml";

// File name used for trigger mode settings retrieval error test
const char* kGetTriggerModeAbnormalTestFileName = "test/config/get_trigger_mode_abnormal_%02d.yml";

// Number of files used for trigger mode settings retrieval error test
const int32_t kGetTriggerModeAbnormalTestNumberOfFiles = 2;

// File name used for trigger delay settings retrieval error test
const char* kGetTriggerDelayAbnormalTestFileName = "test/config/get_trigger_delay_abnormal_%02d.yml";

// Number of files used for trigger delay settings retrieval error test
const int32_t kGetTriggerDelayAbnormalTestNumberOfFiles = 1;

// File name used for RGB conversion flag retrieval error test
const char* kGetImageTypeAbnormalTestFileName = "test/config/get_image_type_abnormal_%02d.yml";

// Number of files used for RGB conversion flag retrieval error test
const int32_t kGetImageTypeAbnormalTestNumberOfFiles = 1;

// Verify whether the parameter combination is valid
const char* kCheckValidParamCombinationFileName = "test/config/check_valid_param_combination_%02d.yml";

// Verify 3.3V output settings retrieval
const char* const kCheckOutputVoltageSettingFileName = "test/config/get_output_voltage_%02d.yml";

}  // anonymous namespace

namespace tmc_pgr_camera {

/// @brief Test fixture for loading configuration files of the Point Grey camera system
class YamlPointGreyCameraSystemSettingTest : public testing::Test {
 protected:
  /// @brief Detect markers from test images
  static void SetUpTestCase() {
    ASSERT_NO_THROW(s_default_setting.reset(new YamlPointGreyCameraSystemSetting(
      kDefaultConfigFileName)))
          << "Failed to read setting file. Path: "
          << std::filesystem::absolute(kDefaultConfigFileName);
  }

  // Default configuration file
  static std::shared_ptr<YamlPointGreyCameraSystemSetting> s_default_setting;
};

std::shared_ptr<YamlPointGreyCameraSystemSetting> YamlPointGreyCameraSystemSettingTest::s_default_setting;

/// @brief Verify failure with unreadable configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, ReadAbnormal) {
  // Load a non-existent file
  ASSERT_FALSE(std::filesystem::exists(kInvalidFilePath));
  ASSERT_ANY_THROW(YamlPointGreyCameraSystemSetting file(kInvalidFilePath));
}

/// @brief Load serial numbers from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetSerialNumbersNormal) {
  std::vector<uint32_t> serial_numbers;
  ASSERT_NO_THROW(serial_numbers = s_default_setting->GetSerialNumbers());

  // Number of cameras is 2
  ASSERT_EQ(2, serial_numbers.size());

  // Serial numbers are being read correctly
  ASSERT_EQ(kDefaultSerialNumber0, serial_numbers.at(0));
  ASSERT_EQ(kDefaultSerialNumber1, serial_numbers.at(1));

  // The camera set as master comes first
  // Here, the second camera is set as master, and the order reversal is verified
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting;
  ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(
      kGetSerialNumbersNormalTestFileName)));
  ASSERT_NO_THROW(serial_numbers = setting->GetSerialNumbers());
  ASSERT_EQ(kDefaultSerialNumber1, serial_numbers.at(0));
  ASSERT_EQ(kDefaultSerialNumber0, serial_numbers.at(1));
}

/// @brief Unable to correctly load serial numbers from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetSerialNumbersAbnormal) {
  // Throw an exception when there is a contradiction in the configuration file
  // 00: Key "cameras" does not exist
  // 01: The "cameras" section is not an array
  // 02: Camera ID does not exist
  // 03: Camera ID is not a numeric value
  // 04: Master setting value is not boolean
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting;
  for (int32_t i = 0; i < kGetSerialNumbersAbnormalTestNumberOfFiles; ++i) {
    std::filesystem::path file =
        std::filesystem::absolute(
           (boost::format(kGetSerialNumbersAbnormalTestFileName) % i).str());
    ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
    ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
    ASSERT_ANY_THROW(setting->GetSerialNumbers()) << "Path: " << file;
  }
}

/// @brief Load properties from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetPropertiesNormal) {
  std::vector<FlyCapture2::Property> properties;
  ASSERT_NO_THROW(properties = s_default_setting->GetProperties());

  // Number of properties is 6
  ASSERT_EQ(6, properties.size());
  for (uint32_t i = 0; i < properties.size(); ++i) {
    const FlyCapture2::Property& property = properties.at(i);
    switch (property.type) {
      case FlyCapture2::SHARPNESS:
        // Verify whether properties are being read correctly
        ASSERT_DOUBLE_EQ(1024.0, property.absValue);
        ASSERT_FALSE(property.absControl);
        ASSERT_EQ(0, property.valueA);
        ASSERT_EQ(0, property.valueB);
        ASSERT_FALSE(property.onOff);
        ASSERT_FALSE(property.autoManualMode);
        ASSERT_TRUE(property.onePush);
        break;

      case FlyCapture2::WHITE_BALANCE:
        ASSERT_EQ(570, property.valueA);
        ASSERT_EQ(810, property.valueB);
        ASSERT_TRUE(property.onOff);
        ASSERT_TRUE(property.autoManualMode);
        ASSERT_FALSE(property.onePush);
        break;
      default:
        break;
    }
  }
}

/// @brief Verify failure in loading properties from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetPropertiesAbnormal) {
  // Throw an exception when there is a contradiction in the configuration file
  // 00: Key name of a non-existent property
  // 01: Key name of a non-existent property setting value
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting;
  for (int32_t i = 0; i < kGetPropertiesAbnormalTestNumberOfFiles; ++i) {
    std::filesystem::path file =
        std::filesystem::absolute(
            (boost::format(kGetPropertiesAbnormalTestFileName) % i).str());
    ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
    ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
    ASSERT_ANY_THROW(setting->GetProperties()) << "Path: " << file;
  }
}

/// @brief Load frame rates from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetFrameRateNormal) {
  std::optional<std::pair<FlyCapture2::FrameRate, float> > frame_rate;

  // Format7 disabled
  ASSERT_NO_THROW(frame_rate = s_default_setting->GetFrameRate(FlyCapture2::VIDEOMODE_640x480Y8));
  ASSERT_TRUE((bool)frame_rate);
  ASSERT_EQ(FlyCapture2::FRAMERATE_7_5, frame_rate->first);
  ASSERT_FLOAT_EQ(7.5, frame_rate->second);

  // Format7 enabled
  ASSERT_NO_THROW(frame_rate = s_default_setting->GetFrameRate(FlyCapture2::VIDEOMODE_FORMAT7));
  ASSERT_TRUE((bool)frame_rate);
  ASSERT_EQ(FlyCapture2::FRAMERATE_FORMAT7, frame_rate->first);
  ASSERT_FLOAT_EQ(7.5, frame_rate->second);
}

/// @brief Failure in loading frame rates from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetFrameRateAbnormal) {
  // Throw an exception when there is a contradiction in the configuration file (Format7 disabled)
  // 00: Unsettable frame rate value
  // 01: Frame rate value is not a numeric value (e.g., string)
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting;
  for (int32_t i = 0; i < kGetFrameRateAbnormalTestNumberOfFiles; ++i) {
    std::filesystem::path file =
        std::filesystem::absolute(
            (boost::format(kGetFrameRateAbnormalTestFileName) % i).str());
    ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
    ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
    ASSERT_ANY_THROW(setting->GetFrameRate(FlyCapture2::VIDEOMODE_640x480Y8)) << "Path: " << file;
  }
}

/// @brief Load video modes from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetVideoModeNormal) {
  std::optional<FlyCapture2::VideoMode> video_mode;
  ASSERT_NO_THROW(video_mode = s_default_setting->GetVideoMode());
  ASSERT_TRUE((bool)video_mode);
  ASSERT_EQ(FlyCapture2::VIDEOMODE_FORMAT7, *video_mode);
}

/// @brief Failure in loading video modes from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetVideoModeAbnormal) {
  // Throw an exception when there is a contradiction in the configuration file
  // 00: Invalid video mode
  // 01: Video mode is not a string
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting;
  for (int32_t i = 0; i < kGetVideoModeAbnormalTestNumberOfFiles; ++i) {
    std::filesystem::path file =
        std::filesystem::absolute(
            (boost::format(kGetVideoModeAbnormalTestFileName) % i).str());
    ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
    ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
    ASSERT_ANY_THROW(setting->GetVideoMode()) << "Path: " << file;
  }
}

/// @brief Load Format7 settings from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetFormat7SettingNormal) {
  std::optional<FlyCapture2::Format7ImageSettings> format7_setting;
  ASSERT_NO_THROW(format7_setting = s_default_setting->GetFormat7Setting());
  ASSERT_TRUE((bool)format7_setting);
  ASSERT_EQ(0, static_cast<int32_t>(format7_setting->mode));
  ASSERT_EQ(1280, format7_setting->width);
  ASSERT_EQ(960, format7_setting->height);
  ASSERT_EQ(8, format7_setting->offsetX);
  ASSERT_EQ(2, format7_setting->offsetY);
  ASSERT_EQ(FlyCapture2::PIXEL_FORMAT_RAW8, format7_setting->pixelFormat);
}

/// @brief Failure in loading Format7 settings from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetFormat7SettingAbnormal) {
  // Throw an exception when there is a contradiction in the configuration file
  // 00: "mode" key does not exist
  // 01: "mode" value is a string
  // 02: "mode" value is out of range (-1)
  // 03: "mode" value is out of range (32)
  // 04: "width" key does not exist
  // 05: "width" value is a string
  // 06: "height" key does not exist
  // 07: "height" value is a string
  // 08: "offsetX" key does not exist
  // 09: "offsetX" value is a string
  // 10: "offsetY" key does not exist
  // 11: "offsetY" value is a string
  // 12: "pixelFormat" key does not exist
  // 13: "pixelFormat" value is numeric
  // 14: "pixelFormat" value is an unsettable string
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting;
  for (int32_t i = 0; i < kGetFormat7SettingAbnormalTestNumberOfFiles; ++i) {
    std::filesystem::path file =
        std::filesystem::absolute(
           (boost::format(kGetFormat7SettingAbnormalTestFileName) % i).str());
    ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
    ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
    ASSERT_ANY_THROW(setting->GetFormat7Setting()) << "Path: " << file;
  }
}

/// @brief Load software demosaicing from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetSoftDemosaicingNormal) {
  std::optional<FlyCapture2::ColorProcessingAlgorithm> soft_demosaicing;
  ASSERT_NO_THROW(soft_demosaicing = s_default_setting->GetSoftDemosaicing());
  ASSERT_TRUE((bool)soft_demosaicing);
  ASSERT_EQ(FlyCapture2::EDGE_SENSING, *soft_demosaicing);
}

/// @brief Failure in loading software demosaicing from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetSoftDemosaicingAbormal) {
  // Throw an exception when there is a contradiction in the configuration file
  // 00: Setting value is numeric
  // 01: Non-existent string value
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting;
  for (int32_t i = 0; i < kGetSoftDemosaicingAbnormalTestNumberOfFiles; ++i) {
    std::filesystem::path file =
        std::filesystem::absolute(
           (boost::format(kGetSoftDemosaicingAbnormalTestFileName) % i).str());
    ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
    ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
    ASSERT_ANY_THROW(setting->GetSoftDemosaicing()) << "Path: " << file;
  }
}

/// @brief Load software triggers from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, IsSoftwareTriggerEnabledNormal) {
  bool enabled = true;
  ASSERT_NO_THROW(enabled = s_default_setting->IsSoftwareTriggerEnabled());
  ASSERT_FALSE(enabled);
}

/// @brief Failure in loading software triggers from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, IsSoftwareTriggerEnabledAbnormal) {
  // Throw an exception when there is a contradiction in the configuration file
  // 00: Setting value is a string
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting;
  for (int32_t i = 0; i < kIsSoftwareTriggerEnabledAbnormalTestNumberOfFiles; ++i) {
    std::filesystem::path file =
        std::filesystem::absolute(
            (boost::format(kIsSoftwareTriggerEnabledAbnormalTestFileName) % i).str());
    ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
    ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
    ASSERT_ANY_THROW(setting->IsSoftwareTriggerEnabled()) << "Path: " << file;
  }
}

/// @brief Load self-triggers from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, IsSelfTriggerEnabledNormal) {
  bool enabled = false;
  ASSERT_NO_THROW(enabled = s_default_setting->IsSelfTriggerEnabled());
  ASSERT_TRUE(enabled);
}

/// @brief Failure in loading self-triggers from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, IsSelfTriggerEnabledAbnormal) {
  // Throw an exception when there is a contradiction in the configuration file
  // 00: Setting value is a string
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting;
  for (int32_t i = 0; i < kIsSelfTriggerEnabledAbnormalTestNumberOfFiles; ++i) {
    std::filesystem::path file =
        std::filesystem::absolute(
            (boost::format(kIsSelfTriggerEnabledAbnormalTestFileName) % i).str());
    ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
    ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
    ASSERT_ANY_THROW(setting->IsSelfTriggerEnabled()) << "Path: " << file;
  }
}

/// @brief Load self-trigger settings from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetSelfTriggerSettingsNormal) {
  std::optional<SelfTriggerSettings> trigger_settings;
  ASSERT_NO_THROW(trigger_settings = s_default_setting->GetSelfTriggerSettings());
  ASSERT_TRUE((bool)trigger_settings);
  ASSERT_EQ(0, trigger_settings->in_io);
  ASSERT_EQ(1, trigger_settings->out_io);
  ASSERT_EQ(0x040000400, trigger_settings->pulse_figure);
  ASSERT_EQ(0x01, trigger_settings->number_of_pulse);
}

/// @brief Failure in loading self-trigger settings from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetSelfTriggerSettingsAbnormal) {
  // Throw an exception when there is a contradiction in the configuration file
  // 00: "in_io" key does not exist
  // 01: "in_io" value is a string
  // 02: "out_io" key does not exist
  // 03: "out_io" value is a string
  // 04: "pulse_figure" key does not exist
  // 05: "pulse_figure" value is a string
  // 06: "number_of_pulse" key does not exist
  // 07: "number_of_pulse" value is a string
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting;
  for (int32_t i = 0; i < kGetSelfTriggerSettingsAbnormalTestNumberOfFiles; ++i) {
    std::filesystem::path file =
        std::filesystem::absolute(
            (boost::format(kGetSelfTriggerSettingsAbnormalTestFileName) % i).str());
    ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
    ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
    ASSERT_ANY_THROW(setting->GetSelfTriggerSettings()) << "Path: " << file;
  }
}

/// @brief Load trigger mode settings from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetTriggerModeNormal) {
  std::optional<FlyCapture2::TriggerMode> trigger_mode;
  ASSERT_NO_THROW(trigger_mode = s_default_setting->GetTriggerMode());
  ASSERT_TRUE((bool)trigger_mode);
  ASSERT_EQ(true, trigger_mode->onOff);
  ASSERT_EQ(0, static_cast<int32_t>(trigger_mode->mode));
  ASSERT_EQ(0, trigger_mode->parameter);
  ASSERT_EQ(0, trigger_mode->source);

  // 00: Self-trigger on, "source" value is reflected
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting;
  std::filesystem::path file =
      std::filesystem::absolute(
          (boost::format(kGetTriggerModeNormalTestFileName) % 0).str());
  ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
  ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
  ASSERT_NO_THROW(trigger_mode = setting->GetTriggerMode()) << "Path: " << file;
  ASSERT_TRUE((bool)trigger_mode) << "Path: " << file;
  ASSERT_EQ(1, trigger_mode->source) << "Path: " << file;

  // 01: Software trigger on
  file = std::filesystem::absolute(
      (boost::format(kGetTriggerModeNormalTestFileName) % 1).str());
  ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
  ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
  ASSERT_NO_THROW(trigger_mode = setting->GetTriggerMode()) << "Path: " << file;
  ASSERT_TRUE((bool)trigger_mode) << "Path: " << file;
  ASSERT_EQ(1, trigger_mode->parameter) << "Path: " << file;
  ASSERT_EQ(7, trigger_mode->source) << "Path: " << file;

  // 02: External trigger
  file = std::filesystem::absolute(
      (boost::format(kGetTriggerModeNormalTestFileName) % 2).str());
  ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
  ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
  ASSERT_NO_THROW(trigger_mode = setting->GetTriggerMode()) << "Path: " << file;
  ASSERT_TRUE((bool)trigger_mode) << "Path: " << file;
  ASSERT_EQ(0, trigger_mode->parameter) << "Path: " << file;
  ASSERT_EQ(0, trigger_mode->source) << "Path: " << file;
}

/// @brief Failure in loading trigger mode settings from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetTriggerModeAbnormal) {
  // Throw an exception when there is a contradiction in the configuration file
  // 00: Self-trigger on but no self-trigger IO settings
  // 01: Inappropriate key exists
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting;
  for (int32_t i = 0; i < kGetTriggerModeAbnormalTestNumberOfFiles; ++i) {
    std::filesystem::path file =
        std::filesystem::absolute(
            (boost::format(kGetTriggerModeAbnormalTestFileName) % i).str());
    ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
    ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
    ASSERT_ANY_THROW(setting->GetTriggerMode()) << "Path: " << file;
  }
}

/// @brief Load trigger delay settings from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetTriggerDelayNormal) {
  std::optional<FlyCapture2::TriggerDelay> trigger_delay;
  ASSERT_NO_THROW(trigger_delay = s_default_setting->GetTriggerDelay());
  ASSERT_TRUE((bool)trigger_delay);
  ASSERT_DOUBLE_EQ(5.0, trigger_delay->absValue);
  ASSERT_EQ(FlyCapture2::TRIGGER_DELAY, trigger_delay->type);
  ASSERT_FALSE(trigger_delay->onOff);
  ASSERT_TRUE(trigger_delay->absControl);
}

/// @brief Failure in loading trigger delay settings from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetTriggerDelayAbnormal) {
  // Throw an exception when there is a contradiction in the configuration file
  // 00: Inappropriate key exists
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting;
  for (int32_t i = 0; i < kGetTriggerDelayAbnormalTestNumberOfFiles; ++i) {
    std::filesystem::path file =
        std::filesystem::absolute(
            (boost::format(kGetTriggerDelayAbnormalTestFileName) % i).str());
    ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
    ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
    ASSERT_ANY_THROW(setting->GetTriggerDelay()) << "Path: " << file;
  }
}

/// @brief Load RGB conversion flags from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetImageTypeNormal) {
  std::optional<ImageType> image_type;
  ASSERT_NO_THROW(image_type = s_default_setting->GetImageType());
  ASSERT_TRUE((bool)image_type);
  ASSERT_TRUE(*image_type == kRgbImage);
}

/// @brief Failure in loading RGB conversion flags from configuration files
TEST_F(YamlPointGreyCameraSystemSettingTest, GetImageTypeAbnormal) {
  // Throw an exception when there is a contradiction in the configuration file
  // 00: Value is not Boolean but a string
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting;
  for (int32_t i = 0; i < kGetImageTypeAbnormalTestNumberOfFiles; ++i) {
    std::filesystem::path file =
        std::filesystem::absolute(
            (boost::format(kGetImageTypeAbnormalTestFileName) % i).str());
    ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
    ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
    ASSERT_ANY_THROW(setting->GetImageType()) << "Path: " << file;
  }
}

TEST_F(YamlPointGreyCameraSystemSettingTest, CheckValidParamCombination) {
  // 0 - 8 Abnormality with self_trigger_io being closed
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting;
  int i = 0;
  for (; i <= 8; ++i) {
    std::filesystem::path file =
        std::filesystem::absolute(
            (boost::format(kCheckValidParamCombinationFileName) % i).str());
    ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
    ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
    ASSERT_ANY_THROW(setting->GetSelfTriggerSettings()) << "Path: " << file;
  }

  // 9 - 12 Inconsistency between self_trigger_io and trigger_mode
  for (; i <= 12; ++i) {
    std::filesystem::path file =
        std::filesystem::absolute(
            (boost::format(kCheckValidParamCombinationFileName) % i).str());
    ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
    ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
    ASSERT_NO_THROW(setting->GetSelfTriggerSettings()) << "Path: " << file;
    ASSERT_ANY_THROW(setting->GetTriggerMode()) << "Path: " << file;
  }

  // 13 - 17 Normal cases
  for (; i < 17; ++i) {
    std::filesystem::path file =
        std::filesystem::absolute(
            (boost::format(kCheckValidParamCombinationFileName) % i).str());
    ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
    ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
    ASSERT_NO_THROW(setting->GetSelfTriggerSettings()) << "Path: " << file;
    ASSERT_NO_THROW(setting->GetTriggerMode()) << "Path: " << file;
  }
}

TEST_F(YamlPointGreyCameraSystemSettingTest, CheckOutputVoltageSetting) {
  // 0 - 1 Abnormal cases
  std::shared_ptr<YamlPointGreyCameraSystemSetting> setting;
  int i = 0;
  for (; i <= 1; ++i) {
    std::filesystem::path file =
        std::filesystem::absolute(
            (boost::format(kCheckOutputVoltageSettingFileName) % i).str());
    ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
    ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
    ASSERT_ANY_THROW(setting->GetOutputVoltageSetting()) << "Path: " << file;
  }

  // 2 Normal case
  std::filesystem::path file =
      std::filesystem::absolute(
          (boost::format(kCheckOutputVoltageSettingFileName) % 2).str());
  ASSERT_TRUE(std::filesystem::exists(file)) << "Path: " << file;
  ASSERT_NO_THROW(setting.reset(new YamlPointGreyCameraSystemSetting(file.string())));
  ASSERT_NO_THROW(setting->GetOutputVoltageSetting()) << "Path: " << file;
}

}  // end of namespace tmc_pgr_camera

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
