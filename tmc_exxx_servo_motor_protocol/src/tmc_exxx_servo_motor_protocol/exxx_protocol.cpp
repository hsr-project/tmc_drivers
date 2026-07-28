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
/// @file exxx_protocol.hpp
/// Communication protocol for Exxx amp
#include <algorithm>
#include <vector>
#include <tmc_exxx_servo_motor_protocol/exxx_protocol.hpp>

namespace tmc_exxx_servo_motor_protocol {

ExxxProtocol::ErrorCode ExxxProtocol::Ping(uint8_t id) {
  ExxxProtocol::ErrorCode error = network_->Send(id, kInstructionPing, NULL, 0);
  // Return system error immediately
  if (error.category() == boost::system::system_category() && error) {
    return error;
  }
  error = network_->Receive(id, receive_buffer_);
  // Return system error immediately
  if (error.category() == boost::system::system_category() && error) {
    return error;
  } else {
    // Ignore anything other than system error
    return ErrorCode(boost::system::errc::success, boost::system::system_category());
  }
}

ExxxProtocol::ErrorCode ExxxProtocol::Reset(uint8_t id) {
  std::array<uint8_t, 9> send_data = {0x52, 0x45, 0x53, 0x45, 0x54, 0x00, 0xFF, 0xAA, 0x55};
  ExxxProtocol::ErrorCode error = network_->Send(id, kInstructionReset, &send_data[0], 9);
  // Return system error immediately
  if (error.category() == boost::system::system_category() && error) {
    return error;
  }
  error = network_->Receive(id, receive_buffer_);
  // Since there is no response on success, consider timeout as normal completion
  if (error.value() == boost::system::errc::timed_out) {
    return ExxxProtocol::ErrorCode(boost::system::errc::success, boost::system::system_category());
  }
  // Return system error immediately
  if (error.category() == boost::system::system_category() && error) {
    return error;
  } else {
    // Return an error upon successful reception
    return ExxxProtocol::ErrorCode(boost::system::errc::bad_message, boost::system::system_category());
  }
}

ExxxProtocol::ErrorCode ExxxProtocol::SyncParam(uint8_t id) {
  ExxxProtocol::ErrorCode error = network_->Send(id, kInstructionSyncParam, NULL, 0);
  // Return system error immediately
  if (error.category() == boost::system::system_category() && error) {
    return error;
  }
  error = network_->Receive(id, receive_buffer_);
  // Return system error immediately
  if (error.category() == boost::system::system_category() && error) {
    return error;
  } else {
    // Ignore anything other than system error
    return ExxxProtocol::ErrorCode(boost::system::errc::success, boost::system::system_category());
  }
}

ExxxProtocol::ErrorCode ExxxProtocol::AvagoAvePos(uint8_t id) {
  ExxxProtocol::ErrorCode error = network_->Send(id, kInstructionAvagoAvePos, NULL, 0);
  // Return system error immediately
  if (error.category() == boost::system::system_category() && error) {
    return error;
  }
  error = network_->Receive(id, receive_buffer_);
  // Return system error immediately
  if (error.category() == boost::system::system_category() && error) {
    return error;
  } else {
    // Ignore anything other than system error
    return ExxxProtocol::ErrorCode(boost::system::errc::success, boost::system::system_category());
  }
}

ExxxProtocol::ErrorCode ExxxProtocol::WriteEeprom(uint8_t id) {
  ExxxProtocol::ErrorCode error = network_->Send(id, kInstructionWriteEeprom, NULL, 0);
  // No response
  return error;
}

/// Retrieve the git hash value of the firmware
/// @param[in] id Node id
/// @param[out] control_table_hash_out md5sum of control_table.csv
/// @param[out] firmware_hash_out Git hash tag of control firmware
/// @retval success Success
/// @retval message_size Invalid size of the retrieved message
/// @retval other System or ExxxCategory error
ExxxProtocol::ErrorCode ExxxProtocol::ReadHash(uint8_t id, std::vector<uint8_t>& control_table_hash_out,
                                               std::vector<uint8_t>& firmware_hash_out) {
  ExxxProtocol::ErrorCode error = network_->Send(id, kInstructionReadHash, NULL, 0);
  // Return system error immediately
  if (error.category() == boost::system::system_category() && error) {
    return error;
  }
  error = network_->Receive(id, receive_buffer_);
  // Return system error immediately
  if (error.category() == boost::system::system_category() && error) {
    return error;
  } else if (receive_buffer_.size() != kControlTableHashByte + kFirmwareHashByte) {
    // Error on receiving invalid size
    return ExxxProtocol::ErrorCode(boost::system::errc::message_size, boost::system::system_category());
  }
  control_table_hash_out.resize(kControlTableHashByte);
  std::copy(&receive_buffer_[0], &receive_buffer_[kControlTableHashByte], control_table_hash_out.begin());

  firmware_hash_out.resize(kFirmwareHashByte);
  std::copy(&receive_buffer_[kControlTableHashByte], &receive_buffer_[kControlTableHashByte + kFirmwareHashByte],
            firmware_hash_out.begin());
  // Ignore anything other than system error
  return ExxxProtocol::ErrorCode(boost::system::errc::success, boost::system::system_category());
}
}  // namespace tmc_exxx_servo_motor_protocol
