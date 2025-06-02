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
/// @brief      Camera Nodelet
#include "tmc_pgr_camera/camera_nodelet.hpp"
#include <string>
#include <vector>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <wordexp.h>
#include <yaml-cpp/yaml.h>
#include "tmc_pgr_camera/camera_images_publisher.hpp"
#include "tmc_pgr_camera/i_camera_system_plugin_base.hpp"
#include "tmc_pgr_camera/image.hpp"
#include "tmc_pgr_camera/ros_parameter_point_grey_camera_system_setting.hpp"
#include "tmc_pgr_camera/yaml_point_grey_camera_system_setting.hpp"
#include "tmc_vision_msgs/srv/control_camera.hpp"

namespace {

// Parameter name for obtaining the frame ID
const char* const kFrameIdParamName = "frame_id";
// Default frame ID
const char* const kDefaultFrameId = "pgr_stereo_camera";

// Parameter name for obtaining the number of cameras to use
const char* const kCameraNumParamName = "camera_num";

// Image topic parameter name
const char* const kImageTopicNames = "image_topic_names";

// Parameter name for obtaining the filename describing camera settings
const char* const kCameraSettingFilePathParamName = "camera_setting_file_path";

// Parameter name for obtaining the flag to start capturing upon launch
const char* const kAutoCaptureStartParamName = "auto_capture_start";
// Default flag to start capturing upon launch
const bool kAutoCaptureStart = true;

// Parameter name for camera settings
const char* const kSettingParamName = "setting";

// Parameter name for camera property settings
const char* const kPropertyParamName = "property";

// Parameter name for obtaining the service name to control the camera
const char* const kControlCamera = "control_camera";

// Base name of the camera system plugin
const char* const kPluginBaseName = "tmc_pgr_camera::ICameraSystemPluginBase";

// ROS parameter name for camera ID
const char* const kCameraParamName = "camera";

// Listed within export in package.xml
// ROS parameter name for obtaining the tag name containing the plugin to be used as a camera
const char* const kPluginExportTagNameParamName = "plugin_export_tag_name";

// ROS parameter name for obtaining the name of the camera plugin to be used
const char* const kPluginNameParamName = "plugin_name";

// The leading key of the parameter group containing brightness, etc., within Yaml
const char* const kYamlKeyParameter = "parameter";

// Key for the camera plugin name within Yaml
const char* const kYamlKeyPlugin = "plugin";

// Parameter name for the number of times to ignore initial frame capture failures directly after launch
const char* const kIgnoreInitialGrabErrorThresholdParameterName = "ignore_initial_grab_error_threshold";

// The number of times to ignore initial frame capture failures directly after launch
// The threshold value of 20 ensures no warnings occur between 1 and 15Hz, regardless of whether executed without stress or with stress --cpu 8
const int kIgnoreInitialGrabErrorThreshold = 20;

// mode1 (Bulb shutter mode) of trigger_mode
const int kTriggerMode1 = 1;
}  // anonymous namespace

namespace tmc_pgr_camera {

/// @brief Constructor
CameraNodelet::CameraNodelet(const rclcpp::NodeOptions& options)
    : Node("camera_node", options),
      camera_loader_(),
      camera_(),
      camera_images_publisher_(),
      control_camera_service_(),
      capture_thread_(),
      can_stop_capture_thread_(false),
      access_(),
      property_changed_(false) {
  onInit();
}

/// @brief Destructor
/// @note Waits until the capture thread ends
CameraNodelet::~CameraNodelet() {
  try {
    {
      std::unique_lock<std::shared_mutex> write(access_);
      can_stop_capture_thread_ = true;
    }
    capture_thread_->join();
  } catch (...) {
    // pass
  }
}

/// @brief Initializes Nodelet
/// @exception std::runtime_error When the camera count setting is not 1 or 2
/// @exception std::runtime_error When the camera settings file cannot be loaded
void CameraNodelet::onInit() {
  // Obtaining frame ID
  std::string frame_id = this->declare_parameter(kFrameIdParamName, kDefaultFrameId);

  // Camera settings class to be loaded by the plugin
  camera_setting_.reset(new RosParameterPointGreyCameraSystemSetting(std::shared_ptr<CameraNodelet>(this)));

  // Read the camera serial number (ID) and upload it to the parameter server
  std::shared_ptr<YamlPointGreyCameraSystemSetting> camera_setting_from_yaml;
  std::string camera_setting_file_path =
      this->declare_parameter(kCameraSettingFilePathParamName, std::string());
  std::vector<int> camera_ids;
  try {
    camera_setting_from_yaml.reset(new YamlPointGreyCameraSystemSetting(camera_setting_file_path));
    const std::vector<uint32_t> camera_ids_uint = camera_setting_from_yaml->GetSerialNumbers();
    for (std::vector<uint32_t>::const_iterator it = camera_ids_uint.begin(); it != camera_ids_uint.end(); ++it) {
      camera_ids.push_back(static_cast<int>(*it));
    }
    // Register with the parameter server
    this->declare_parameter(std::string(kCameraParamName), camera_ids);
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    throw;
  }

  // Register the distributor of camera images
  if (camera_ids.size() <= 0) {
    const std::string error_description("Invalid_camera num.");
    RCLCPP_ERROR(this->get_logger(), "%s", error_description.c_str());
    throw std::runtime_error(error_description.c_str());
  } else if (camera_ids.size() > 2) {
    const std::string error_description("This node supports only stereo or monocular camera.");
    RCLCPP_ERROR(this->get_logger(), "%s", error_description.c_str());
    throw std::runtime_error(error_description.c_str());
  }

  std::vector<std::string> image_topic_names =
      this->declare_parameter(kImageTopicNames, std::vector<std::string>());

  if (image_topic_names.size() != camera_ids.size()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Number of image topics(%d) and number of cameras(%d) do not match.",
                 static_cast<int>(image_topic_names.size()),
                 static_cast<int>(camera_ids.size()));
    throw;
  }
  camera_images_publisher_.reset(
      new CameraImagesPublisher(std::shared_ptr<CameraNodelet>(this), image_topic_names, frame_id));

  // The method of setting parameters other than serial using the previously used yml files (e.g. stereo_pgr_camera.yml) has been abolished.
  // Notify the user if parameters other than serial may exist.
  // Expand ~ using $HOME
  wordexp_t expanded_result;
  ::wordexp(camera_setting_file_path.c_str(), &expanded_result, 0);
  const std::string camera_setting_file_full_path(expanded_result.we_wordv[0]);
  ::wordfree(&expanded_result);
  try {
    const YAML::Node setting_root = YAML::LoadFile(camera_setting_file_full_path);
    // Check if a key of the abolished setting exists
    const YAML::Node& parameter = setting_root[kYamlKeyParameter];
    const YAML::Node& plugin = setting_root[kYamlKeyPlugin];
    if (parameter || plugin) {
      RCLCPP_WARN(this->get_logger(),
                  "Parameters excluding camera ids defined in a yaml file (ex. stereo_pgr_camera.yml) are deprecated.\n"
                  "Those parameters are defined in capture.launch and uploaded to ros parameter server.");
      RCLCPP_WARN(this->get_logger(),
                  "Type a command as below to update.\n"
                  "$ rosrun tmc_pgr_camera update_config %s\n"
                  "If permission denied, execute directly as root like this.\n"
                  "$ sudo /opt/ros/kinetic/lib/tmc_pgr_camera/update_config %s",
                  camera_setting_file_full_path.c_str(),
                  camera_setting_file_full_path.c_str());
    }
  } catch (const YAML::BadFile& e) {
    RCLCPP_ERROR_STREAM(
        this->get_logger(), "Failed to read file. File: " << camera_setting_file_path);
    throw;
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Unknown error has occurred.");
    throw;
  }

  // In ROS 2, get_parameter cannot be performed without explicitly declaring the parameter with declare_parameter.
  // Declare all parameters specified in capture.launch here.
  // Use the initial values from capture.launch.
  this->declare_parameter("property.brightness.on_off", true);
  this->declare_parameter("property.brightness.abs_value", 0.0);
  this->declare_parameter("property.brightness.one_push", true);
  this->declare_parameter("property.brightness.auto_manual_mode", false);

  this->declare_parameter("property.auto_exposure.on_off", true);
  this->declare_parameter("property.auto_exposure.abs_value", 0.0);
  this->declare_parameter("property.auto_exposure.one_push", true);
  this->declare_parameter("property.auto_exposure.auto_manual_mode", true);

  this->declare_parameter("property.white_balance.on_off", true);
  this->declare_parameter("property.white_balance.value_a", 570);
  this->declare_parameter("property.white_balance.value_b", 810);
  this->declare_parameter("property.white_balance.one_push", true);
  this->declare_parameter("property.white_balance.auto_manual_mode", true);

  this->declare_parameter("property.shutter.on_off", true);
  this->declare_parameter("property.shutter.abs_value", 20.0);
  this->declare_parameter("property.shutter.one_push", false);
  this->declare_parameter("property.shutter.auto_manual_mode", false);

  this->declare_parameter("property.gain.on_off", true);
  this->declare_parameter("property.gain.abs_value", 0.0);
  this->declare_parameter("property.gain.one_push", false);
  this->declare_parameter("property.gain.auto_manual_mode", false);

  this->declare_parameter("property.trigger_delay.on_off", false);
  this->declare_parameter("property.trigger_delay.abs_value", 5.0);
  this->declare_parameter("property.trigger_delay.abs_control", true);

  this->declare_parameter("format7.mode", 0);
  this->declare_parameter("format7.offset_x", 8);
  this->declare_parameter("format7.offset_y", 2);
  this->declare_parameter("format7.width", 1280);
  this->declare_parameter("format7.height", 960);
  this->declare_parameter("format7.pixel_format", "raw8");

  this->declare_parameter("self_trigger.io", std::vector<int64_t>{0, 1});
  this->declare_parameter("self_trigger.pulse_width", std::vector<int64_t>{1, 16});
  this->declare_parameter("self_trigger.number_of_pulse", 1);
  this->declare_parameter("self_trigger.polarity", 0);

  this->declare_parameter("software_demosaicing", "edge_sensing");
  this->declare_parameter("video_mode", "format7");
  this->declare_parameter("frame_rate", 5.0);
  this->declare_parameter("trigger_mode.mode", 0);
  this->declare_parameter("trigger_mode.on_off", true);
  this->declare_parameter("trigger_mode.polarity", 0);
  this->declare_parameter("change_rgb_flag", true);
  this->declare_parameter("output_voltage", false);

  // Camera object creation and launch
  std::string export_name =
      this->declare_parameter(kPluginExportTagNameParamName, "tmc_pgr_camera");
  std::string plugin_name =
      this->declare_parameter(kPluginNameParamName, "tmc_pgr_camera/point_grey_camera_system");
  try {
    camera_loader_.reset(new pluginlib::ClassLoader<ICameraSystemPluginBase>(export_name, kPluginBaseName));
    camera_ = camera_loader_->createSharedInstance(plugin_name);
    camera_->Initialize(camera_setting_);
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    throw;
  }

  // Check whether to start capturing immediately upon capture thread initiation
  bool auto_capture_start =
      this->declare_parameter(kAutoCaptureStartParamName, kAutoCaptureStart);
  if (auto_capture_start) {
    camera_->StartCapture();
  }

  // Register the service that controls the capture start/stop commands
  control_camera_service_ = this->create_service<tmc_vision_msgs::srv::ControlCamera>(
      kControlCamera,
      std::bind(&CameraNodelet::ControlCameraCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Capture thread initiation
  capture_thread_.reset(new std::thread(std::bind(&CameraNodelet::CaptureThread, this)));

  const std::string& ns = std::string(kPropertyParamName) + std::string(".");
  this->declare_parameter(ns + std::string("brightness_abs_value"), 1.367188);

  this->declare_parameter(ns + std::string("auto_exposure_one_push"), true);
  this->declare_parameter(ns + std::string("auto_exposure_on_off"), true);
  this->declare_parameter(ns + std::string("auto_exposure_auto_manual_mode"), true);
  this->declare_parameter(ns + std::string("auto_exposure_abs_value"), 0.0);

  this->declare_parameter(ns + std::string("white_balance_one_push"), true);
  this->declare_parameter(ns + std::string("white_balance_on_off"), true);
  this->declare_parameter(ns + std::string("white_balance_auto_manual_mode"), false);
  this->declare_parameter(ns + std::string("white_balance_value_a"), 570);
  this->declare_parameter(ns + std::string("white_balance_value_b"), 810);

  this->declare_parameter(ns + std::string("shutter_one_push"), false);
  this->declare_parameter(ns + std::string("shutter_auto_manual_mode"), false);
  this->declare_parameter(ns + std::string("shutter_abs_value"), 20.0);

  this->declare_parameter(ns + std::string("gain_one_push"), false);
  this->declare_parameter(ns + std::string("gain_auto_manual_mode"), false);
  this->declare_parameter(ns + std::string("gain_abs_value"), 0.0);

  this->declare_parameter(ns + std::string("trigger_mode_on_off"), true);
  this->declare_parameter(ns + std::string("trigger_mode_polarity"), 0);
  this->declare_parameter(ns + std::string("trigger_mode_mode"), 0);

  this->declare_parameter(ns + std::string("trigger_delay_abs_control"), true);
  this->declare_parameter(ns + std::string("trigger_delay_on_off"), false);
  this->declare_parameter(ns + std::string("trigger_delay_value_a"), 307);
  this->declare_parameter(ns + std::string("trigger_delay_abs_value"), 5.0);

  this->declare_parameter(ns + std::string("frame_rate_abs_control"), false);
  this->declare_parameter(ns + std::string("frame_rate_on_off"), false);
  this->declare_parameter(ns + std::string("frame_rate_auto_manual_mode"), false);
  this->declare_parameter(ns + std::string("frame_rate_value_a"), 480);
  this->declare_parameter(ns + std::string("frame_rate_abs_value"), 5.0);

  set_parameters_handle_ = this->add_on_set_parameters_callback(
      std::bind(&CameraNodelet::SetParameterCallback, this, std::placeholders::_1));
}

/// @brief Capture thread
/// @exception std::runtime_error When the pointer of the camera system is null
void CameraNodelet::CaptureThread() {
  if (!camera_) {
    throw std::runtime_error("Camera system does not exist.");
  }

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Start Capture Thread. Capture Status: " << (camera_->IsCapturing() ? "Run" : "Stop"));

  while (rclcpp::ok()) {
    try {
      if (camera_->IsCapturing()) {
        std::optional<std::vector<ImagePtr> > images;
        images = camera_->GrabImage();
        if (!images) {
          continue;
        }
        camera_images_publisher_->Publish(*images);
      }

      if (property_changed_) {
        ChangeCameraProperties();
        camera_->SetSettings(camera_properties_);
        camera_properties_.reset();
        property_changed_ = false;
      }
      std::this_thread::yield();
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1, "%s", e.what());
    }

    std::shared_lock<std::shared_mutex> read(access_);
    if (can_stop_capture_thread_) {
      break;
    }
  }
}

/// @brief Receive capture start/stop commands from HMI
/// @param[in] req Capture start/stop command
/// @param[out] res Success/failure flag of the capture start/stop command
/// @return Flag indicating whether the callback function was successful (always returns true)
/// @exception std::runtime_error When the pointer of the camera system is null
bool CameraNodelet::ControlCameraCallback(
    const std::shared_ptr<tmc_vision_msgs::srv::ControlCamera::Request> req,
    std::shared_ptr<tmc_vision_msgs::srv::ControlCamera::Response> res) {
  if (!camera_) {
    throw std::runtime_error("Camera system does not exist.");
  }
  res->is_success = false;

  // Branch based on whether the command is start or stop
  if (req->capture && !camera_->IsCapturing()) {
    // Start image acquisition
    camera_->StartCapture();
    RCLCPP_INFO(this->get_logger(), "SUCCESS: Start Capture.");
    res->is_success = true;
  } else if (!req->capture && camera_->IsCapturing()) {
    // Stop image acquisition
    camera_->StopCapture();
    RCLCPP_INFO(this->get_logger(), "SUCCESS: Stop Capture.");
    res->is_success = true;
  }

  return true;
}

/// @brief Converts camera settings from parameter to YAML and waits for settings changes
void CameraNodelet::ChangeCameraProperties() {
  const std::string& ns = std::string(kPropertyParamName) + std::string(".");

  // TODO(xx) : define a function and refactoring
  YAML::Node camera_property;

  camera_properties_.reset();

  // brightness
  camera_property["type"] = std::string("brightness");
  camera_property["absValue"] = this->get_parameter(ns + std::string("brightness_abs_value")).get_value<double>();
  camera_properties_.push_back(camera_property);
  camera_property.reset();

  // auto_exposure
  camera_property["type"] = std::string("auto_exposure");
  camera_property["onePush"] = this->get_parameter(ns + std::string("auto_exposure_one_push")).get_value<bool>();
  this->set_parameter(rclcpp::Parameter(ns + std::string("auto_exposure_one_push"), false));
  camera_property["onOff"] = this->get_parameter(ns + std::string("auto_exposure_on_off")).get_value<bool>();
  camera_property["autoManualMode"] =
      this->get_parameter(ns + std::string("auto_exposure_auto_manual_mode")).get_value<bool>();
  camera_property["absValue"] = this->get_parameter(ns + std::string("auto_exposure_abs_value")).get_value<double>();
  camera_properties_.push_back(camera_property);
  camera_property.reset();

  // white_balance
  camera_property["type"] = std::string("white_balance");
  camera_property["onePush"] = this->get_parameter(ns + std::string("white_balance_one_push")).get_value<bool>();
  this->set_parameter(rclcpp::Parameter(ns + std::string("white_balance_one_push"), false));
  camera_property["onOff"] = this->get_parameter(ns + std::string("white_balance_on_off")).get_value<bool>();
  camera_property["autoManualMode"] =
      this->get_parameter(ns + std::string("white_balance_auto_manual_mode")).get_value<bool>();
  camera_property["valueA"] = this->get_parameter(ns + std::string("white_balance_value_a")).get_value<int64_t>();
  camera_property["valueB"] = this->get_parameter(ns + std::string("white_balance_value_b")).get_value<int64_t>();
  camera_properties_.push_back(camera_property);
  camera_property.reset();

  // shutter
  camera_property["type"] = std::string("shutter");
  camera_property["onePush"] = this->get_parameter(ns + std::string("shutter_one_push")).get_value<bool>();
  this->set_parameter(rclcpp::Parameter(ns + std::string("shutter_one_push"), false));
  // Setting mode to 1 (Bulb Shutter mode) in trigger_mode disables automatic shutter mode
  if (this->get_parameter(ns + std::string("trigger_mode_mode")).get_value<int64_t>() == kTriggerMode1) {
    this->set_parameter(rclcpp::Parameter(ns + std::string("shutter_auto_manual_mode"), false));
  }
  camera_property["autoManualMode"] =
      this->get_parameter(ns + std::string("shutter_auto_manual_mode")).get_value<bool>();
  camera_property["absValue"] = this->get_parameter(ns + std::string("shutter_abs_value")).get_value<double>();
  camera_properties_.push_back(camera_property);
  camera_property.reset();

  // gain
  camera_property["type"] = std::string("gain");
  camera_property["onePush"] = this->get_parameter(ns + std::string("gain_one_push")).get_value<bool>();
  this->set_parameter(rclcpp::Parameter(ns + std::string("gain_one_push"), false));
  camera_property["autoManualMode"] = this->get_parameter(ns + std::string("gain_auto_manual_mode")).get_value<bool>();
  camera_property["absValue"] = this->get_parameter(ns + std::string("gain_abs_value")).get_value<double>();
  camera_properties_.push_back(camera_property);
  camera_property.reset();

  // trigger_mode
  camera_property["type"] = std::string("trigger_mode");
  camera_property["onOff"] = this->get_parameter(ns + std::string("trigger_mode_on_off")).get_value<bool>();
  camera_property["polarity"] = this->get_parameter(ns + std::string("trigger_mode_polarity")).get_value<int64_t>();
  camera_property["mode"] = this->get_parameter(ns + std::string("trigger_mode_mode")).get_value<int64_t>();
  camera_properties_.push_back(camera_property);
  camera_property.reset();

  // trigger_delay
  camera_property["type"] = std::string("trigger_delay");
  camera_property["absControl"] = this->get_parameter(ns + std::string("trigger_delay_abs_control")).get_value<bool>();
  camera_property["onOff"] = this->get_parameter(ns + std::string("trigger_delay_on_off")).get_value<bool>();
  camera_property["valueA"] = this->get_parameter(ns + std::string("trigger_delay_value_a")).get_value<int64_t>();
  camera_property["absValue"] = this->get_parameter(ns + std::string("trigger_delay_abs_value")).get_value<double>();
  camera_properties_.push_back(camera_property);
  camera_property.reset();

  // frame_rate
  // Setting on_off to on (1) in trigger_mode results in asynchronous trigger mode, preventing frame_rate setting
  if (this->get_parameter(ns + std::string("trigger_mode_on_off")).get_value<bool>()) {
    this->set_parameter(rclcpp::Parameter(ns + std::string("frame_rate_on_off"), false));
    this->set_parameter(rclcpp::Parameter(ns + std::string("frame_rate_auto_manual_mode"), false));
  } else {
    camera_property["type"] = std::string("frame_rate");
    camera_property["absControl"] = this->get_parameter(ns + std::string("frame_rate_abs_control")).get_value<bool>();
    camera_property["onOff"] = this->get_parameter(ns + std::string("frame_rate_on_off")).get_value<bool>();
    camera_property["autoManualMode"] =
        this->get_parameter(ns + std::string("frame_rate_auto_manual_mode")).get_value<bool>();
    camera_property["valueA"] = this->get_parameter(ns + std::string("frame_rate_value_a")).get_value<int64_t>();
    camera_property["absValue"] = this->get_parameter(ns + std::string("frame_rate_abs_value")).get_value<double>();
    camera_properties_.push_back(camera_property);
    camera_property.reset();
  }
}

rcl_interfaces::msg::SetParametersResult CameraNodelet::SetParameterCallback(
  const std::vector<rclcpp::Parameter>& params) {
    const std::string& ns = std::string(kPropertyParamName) + std::string(".");
    // NOTE: Search due to the excessive number of target parameters.
    std::vector<std::string> property_param_names = {
      ns + std::string("brightness_abs_value"),
      ns + std::string("auto_exposure_on_off"),
      ns + std::string("auto_exposure_auto_manual_mode"),
      ns + std::string("auto_exposure_abs_value"),
      ns + std::string("white_balance_on_off"),
      ns + std::string("white_balance_auto_manual_mode"),
      ns + std::string("white_balance_value_a"),
      ns + std::string("white_balance_value_b"),
      // ns + std::string("shutter_auto_manual_mode"),
      ns + std::string("shutter_abs_value"),
      ns + std::string("gain_auto_manual_mode"),
      ns + std::string("gain_abs_value"),
      ns + std::string("trigger_mode_on_off"),
      ns + std::string("trigger_mode_polarity"),
      ns + std::string("trigger_mode_mode"),
      ns + std::string("trigger_delay_abs_control"),
      ns + std::string("trigger_delay_on_off"),
      ns + std::string("trigger_delay_value_a"),
      ns + std::string("trigger_delay_abs_value"),
      ns + std::string("frame_rate_abs_control"),
      // ns + std::string("frame_rate_on_off"),
      // ns + std::string("frame_rate_auto_manual_mode"),
      ns + std::string("frame_rate_value_a"),
      ns + std::string("frame_rate_abs_value")
    };

    // NOTE: Some parameters might be rewritten within the setting function, and
    //  This callback might be invoked again.
    //  Avoid infinite loops from multiple invocations.

    // Do not invoke settings if onePush is rewritten from true to false.
    std::vector<std::string> property_one_push_names = {
      ns + std::string("auto_exposure_one_push"),
      ns + std::string("white_balance_one_push"),
      ns + std::string("shutter_one_push"),
      ns + std::string("gain_one_push")
    };

    for (const auto& param : params) {
      // Normally, invoke settings if the property-related param changes.
      if (std::find(property_param_names.begin(),
                    property_param_names.end(),
                    param.get_name())
          != property_param_names.end()) {
        property_changed_ = true;
      } else if (std::find(property_one_push_names.begin(),
                           property_one_push_names.end(),
                           param.get_name())
                 != property_one_push_names.end()) {
        // Invoke settings only when onePush is true.
        property_changed_ = (property_changed_ || param.get_value<bool>());
      } else if (param.get_name() == ns + std::string("shutter_auto_manual_mode")) {
        // Parameters rewritten depending on the state of other parameters
        // When mode is 1 (Bulb Shutter mode) in trigger_mode, shutter_auto_manual_mode is rewritten
        property_changed_ = (property_changed_ ||
            this->get_parameter(ns + std::string("trigger_mode_mode")).get_value<int64_t>() != kTriggerMode1);
      } else if (param.get_name() == ns + std::string("frame_rate_on_off") ||
                 param.get_name() == ns + std::string("frame_rate_auto_manual_mode")) {
        // When on_off is on (1) in trigger_mode, frame_rate_on_off and frame_rate_auto_manual_mode are rewritten
        property_changed_ = (property_changed_ ||
            !this->get_parameter(ns + std::string("trigger_mode_on_off")).get_value<bool>());
      }
    }

    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    return result;
}

}  // end of namespace tmc_pgr_camera

RCLCPP_COMPONENTS_REGISTER_NODE(tmc_pgr_camera::CameraNodelet)
