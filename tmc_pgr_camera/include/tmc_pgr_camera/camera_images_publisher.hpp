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
#ifndef TMC_PGR_CAMERA_CAMERA_IMAGES_PUBLISHER_HPP_
#define TMC_PGR_CAMERA_CAMERA_IMAGES_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tmc_pgr_camera/image.hpp"

namespace tmc_pgr_camera {

/// @brief Class that distributes camera images
class CameraImagesPublisher {
  CameraImagesPublisher(const CameraImagesPublisher&) = delete;
  CameraImagesPublisher& operator=(const CameraImagesPublisher&) = delete;

 public:
  /// Constructor
  CameraImagesPublisher(const rclcpp::Node::SharedPtr& node_handle,
                        const std::vector<std::string>& topic_names,
                        const std::string& frame_id);
  /// Distribute the image
  void Publish(const std::vector<ImagePtr>& camera_images) const;

 private:
  /// For obtaining the timestamp
  rclcpp::Clock::SharedPtr clock_;
  /// frame_id of the image message to be distributed
  std::string frame_id_;
  /// Publisher of camera images
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  std::vector<image_transport::Publisher> camera_image_publishers_;
};

}  // end of namespace tmc_pgr_camera

#endif  // TMC_PGR_CAMERA_CAMERA_IMAGES_PUBLISHER_HPP_
