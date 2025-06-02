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
/// @brief Class to create messages
#ifndef TMC_EXXX_SERVO_MOTOR_PROTOCOL_MESSAGE_BUILDER_HPP_
#define TMC_EXXX_SERVO_MOTOR_PROTOCOL_MESSAGE_BUILDER_HPP_

#include <string>

namespace tmc_exxx_servo_motor_protocol {

/// Class to create messages like description (namespace/name)
/// If there are multiple, connect with ";"
class MessageBuilder {
 public:
  /// When there is no namespace
  MessageBuilder() : dst_message_(""), space_("") {}

  /// When there is a namespace
  /// @param [in] space Message namespace, "/" is not needed
  explicit MessageBuilder(const std::string& space) : dst_message_(""), space_(space + "/") {}

  /// Destructor
  ~MessageBuilder() {}

  /// Add message
  /// @param [in] name Name
  /// @param [in] message Description
  void Append(const std::string& name, const std::string& message) {
    dst_message_.append(message);
    dst_message_.append("(");
    dst_message_.append(space_);
    dst_message_.append(name);
    dst_message_.append("); ");
  }

  /// Generate message
  /// @param
  std::string Build() const { return dst_message_; }

 private:
  // Created message
  std::string dst_message_;
  // Namespace
  std::string space_;
};

}  // namespace tmc_exxx_servo_motor_protocol
#endif  // TMC_EXXX_SERVO_MOTOR_PROTOCOL_MESSAGE_BUILDER_HPP_
