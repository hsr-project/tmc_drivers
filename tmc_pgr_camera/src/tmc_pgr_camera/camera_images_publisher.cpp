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
/// @brief      Camera image distributor
#include "tmc_pgr_camera/camera_images_publisher.hpp"
#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace tmc_pgr_camera {

/// @brief Constructor
/// @param[in] node Node handle
/// @param[in] image_topic_names Array of topic names for distributing images (one per camera)
/// @param frame_id frame_id of the image message to be distributed
CameraImagesPublisher::CameraImagesPublisher(const rclcpp::Node::SharedPtr& node_handle,
                                             const std::vector<std::string>& topic_names,
                                             const std::string& frame_id)
    : frame_id_(frame_id), camera_image_publishers_() {
  clock_ = node_handle->get_clock();

  image_transport_.reset(new image_transport::ImageTransport(node_handle));

  switch (static_cast<int>(topic_names.size())) {
  case 1:
    camera_image_publishers_.push_back(image_transport_->advertise(topic_names.at(0).c_str(), 1));
    break;
  case 2:
    camera_image_publishers_.push_back(image_transport_->advertise(topic_names.at(0).c_str(), 1));
    camera_image_publishers_.push_back(image_transport_->advertise(topic_names.at(1).c_str(), 1));
    break;
  default:
    throw std::runtime_error("This publisher supports only stereo or monocular camera.");
  }
}

/// @brief Distribute images
/// @param[in] camera_images Array of camera images to be distributed
/// @exception std::runtime_error If the number of camera images does not match the number of publishers
/// @exception std::runtime_error If no camera images exist
void CameraImagesPublisher::Publish(const std::vector<ImagePtr>& camera_images) const {
  if (camera_images.size() < camera_image_publishers_.size()) {
    throw std::runtime_error(
        "Failed to publish camera images.\n"
        "Number of images is less than number of publishers.");
  }
  if (camera_images.size() > camera_image_publishers_.size()) {
    throw std::runtime_error(
        "Failed to publish camera images.\n"
        "Number of images is larger than number of publishers.");
  }

  if (camera_images.empty()) {
    throw std::runtime_error("There are no images for publishing.");
  }

  // Convert CameraImage to sensor_msgs::Image and distribute
  const rclcpp::Time timestamp = clock_->now();
  const std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
  for (size_t i = 0, size = camera_images.size(); i < size; ++i) {
    const ImagePtr& camera_image = camera_images.at(i);
    if (!camera_image) {
      throw std::runtime_error("Camera image for publishing is invalid.");
    }
    std::string encoding = sensor_msgs::image_encodings::RGB8;
    if (camera_image->image.type() == CV_8UC1) {
      encoding = sensor_msgs::image_encodings::MONO8;
    }
    sensor_msgs::msg::Image::SharedPtr sensor_image =
        cv_bridge::CvImage(std_msgs::msg::Header(), encoding, camera_image->image).toImageMsg();

    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - camera_image->time).count();
    rclcpp::Duration ros_duration(static_cast<int32_t>(duration / 1000000000),
                                  static_cast<uint32_t>(duration) % 1000000000);
    rclcpp::Time image_timestamp = timestamp - ros_duration;
    sensor_image->header.stamp = image_timestamp;
    sensor_image->header.frame_id = frame_id_;
    camera_image_publishers_.at(i).publish(sensor_image);
  }
}

}  // end of namespace tmc_pgr_camera
