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
/// @file control_table.cpp
/// @brief A class that retrieves the control table from a CSV file and uses it for communication
#ifndef TMC_EXXX_SERVO_MOTOR_PROTOCOL_CONTROL_TABLE_HPP_
#define TMC_EXXX_SERVO_MOTOR_PROTOCOL_CONTROL_TABLE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <boost/cstdint.hpp>
#include <boost/utility.hpp>

namespace tmc_exxx_servo_motor_protocol {

class ControlTableItemDescriptor;

class ControlTable : private boost::noncopyable {
 public:
  /// Return value of Load
  enum ErrorCode {
    kSuccess = 0,      /// Success
    kFileOpenError,    /// Failed to open the file
    kColumnSizeError,  /// Error in the number of elements in the control table
    kAlreadyRecorded,  /// More than one identical entry exists
    kBadType,          /// Incorrect type specified
  };

  ControlTable();

  ~ControlTable();

  /// Calculate the md5sum by providing the definition file
  /// @return Success or failure of file reading, success with kSuccess
  ErrorCode CalculateMd5Sum(const std::string& definition_file);

  /// Initialize by providing the definition file
  /// @return Success or failure of file reading, success with kSuccess
  ErrorCode Load(const std::string& definition_file);

  /// Retrieve the md5sum of this control table
  std::vector<uint8_t> GetMd5Sum();

  /// Retrieve properties from the entry name of the control table
  /// Returns an empty shared_ptr if a non-existent name is given
  std::shared_ptr<ControlTableItemDescriptor> ReferItemDescriptor(const std::string& entry) const;

  /// Search for the index corresponding to the command name.
  int GetCommandIndex(const std::string& command_name) const;

  /// Search for the command name corresponding to the index.
  std::string& GetCommandName(const int command_index) const;


 private:
  /// Implementation
  class ControlTableImpl;

  /// pimpl
  std::unique_ptr<ControlTableImpl> pimpl_;
};

}  // namespace tmc_exxx_servo_motor_protocol
#endif  // TMC_EXXX_SERVO_MOTOR_PROTOCOL_CONTROL_TABLE_HPP_
