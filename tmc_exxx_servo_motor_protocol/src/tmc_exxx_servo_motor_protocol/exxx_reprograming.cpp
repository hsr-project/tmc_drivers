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
#include <algorithm>
#include <iomanip>
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <rclcpp/rclcpp.hpp>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <tmc_exxx_servo_motor_protocol/cti485.hpp>
#include <tmc_exxx_servo_motor_protocol/exxx_reprograming.hpp>

namespace {
const uint32_t kSendCommandMaxRetryCount      = 10;                 /// Maximum retry count for command transmission
const uint32_t kFlushXmodemMaxRetryCount      = 100;                /// Maximum retry count for XMODEM communication reception
const uint8_t  kEraseCommand                  = 0x65;               /// Erase command 'e'
const uint8_t  kFlushCommand                  = 0x77;               /// Firmware write command 'w'
const uint8_t  kRunCommand                    = 0x67;               /// Firmware start command 'g'
const uint8_t  kExecuteCommand                = 0x79;               /// Command execution 'y'
const char*    kBootMessage                   = "Bootloader";       /// PROP amp boot section startup message
const char*    kEraseCheckMessage             = "ERASE ALL FLASH";  /// Erase confirmation message
const char*    kEraseFinishMessage            = "COMPLETED";        /// Erase completion message
const char*    kFlushCheckMessage             = "START UPLOAD";     /// Firmware write confirmation message
const char*    kFlushFinishMessage            = "SUCCESS";          /// Firmware write completion message
const char*    kRunCheckMessage               = "GO";               /// Firmware start message
const uint8_t  kXmodemSOH                     = 0x01;               /// XMODEM communication block start
const uint8_t  kXmodemEOT                     = 0x04;               /// XMODEM communication transfer end
const uint8_t  kXmodemACK                     = 0x06;               /// XMODEM communication acknowledgment
const uint8_t  kXmodemNAK                     = 0x15;               /// XMODEM communication request for transmission and negative acknowledgment
const uint8_t  kXmodemCAN                     = 0x18;               /// XMODEM communication cancel
const uint8_t  kXmodemEOF                     = 0xFF;               /// XMODEM communication padding
const uint32_t kXmodemSendSize                = 132;                /// XMODEM communication data transmission size
const uint32_t kXmodemDataSize                = 128;                /// XMODEM communication data section size
const uint32_t kXmodemHeaderSize              = 3;                  /// XMODEM communication header + block number size
const uint32_t kXmodemHeaderPos               = 0;                  /// XMODEM communication header position in transmission data
const uint32_t kXmodemBlockNumberPos          = 1;                  /// XMODEM communication block number position in transmission data
const uint32_t kXmodemBlockNumberCompPos      = 2;                  /// XMODEM communication complement of block number position in transmission data
const uint32_t kXmodemCheckSumPos             = 131;                /// XMODEM communication checksum position in transmission data
const int64_t  kBootStartUpWaitTime           = 3000000000;         /// PROP amp boot section startup wait time [ns]
const int64_t  kReproIntervalTime             = 1000000000;         /// PROP amp startup wait '.' output interval [ns]
const int64_t  kCommandIntervalTime           = 50000000;           /// Command transmission interval [ns]
const int64_t  kEraseWaitTime                 = 1000000000;         /// Erase completion wait time [ns]
const int64_t  kFlushIntervalTime             = 10000000;           /// Firmware write XMODEM communication reception interval [ns]
const uint8_t  kReceiveLineDelimiter1         = 0x0A;               /// '\n'(line feed)
const uint8_t  kReceiveLineDelimiter2         = 0x3F;               /// '?'

/**
 * @brief Wait for a specified time [ns]
 *
 * @param[in]  nsec         Wait time [ns]
 * @return
 * boost::system::errc::success  Successfully waited for the specified time
 * boost::system::errc::invalid_argument  Failed to wait for the specified time
 */
boost::system::error_code WaitNanoSec(int64_t nsec) {
  boost::system::error_code error(boost::system::errc::success, boost::system::system_category());
  timespec duration;
  duration.tv_sec  = nsec / 1000000000LL;
  duration.tv_nsec = nsec % 1000000000LL;

  while (clock_nanosleep(CLOCK_MONOTONIC, 0, &duration, &duration)) {
    if (errno == EINTR) {
      continue;
    } else {
      // EFAULT or EINVAL
      // Both occur due to arguments of the clock_nanosleep function, so set the return value to invalid_argument
      error = boost::system::error_code(boost::system::errc::invalid_argument, boost::system::system_category());
      break;
    }
  }
  return error;
}
}  // anonymous namespace

namespace tmc_exxx_servo_motor_protocol {

/**
 * @brief Constructor
 *
 * @param[in]   device_name   Device name
 * @param[in]   is_usb_rs485  USB usage (true: USB used, false: USB not used)
 * @param[in]   timeout       Send/receive timeout time [ns]
 * @param[in]   sleep_tick    Retry wait time [ns] when the write function returns EAGAIN
 * @retval  void
 */
ExxxReprograming::ExxxReprograming(const std::string& device_name, bool is_usb_rs485, uint32_t baudrate,
                         int64_t timeout, int64_t sleep_tick)
    : fd_(-1), device_name_(device_name), is_usb_rs485_(is_usb_rs485), baudrate_(baudrate),
      timeout_(timeout), sleep_tick_(sleep_tick) {}

/**
 * @brief Destructor
 *
 * Close the file descriptor (communication device)
 *
 * @param   void
 * @retval  void
 */
ExxxReprograming::~ExxxReprograming() { (void)close(fd_); }

/**
 * @brief Open the communication device
 *
 * @return
 * boost::system::errc::success  Device opened successfully
 * Return errors that occur with open, tcgetattr, tcsetattr, ioctl, tcflush, cfsetispeed, tcsetattr
 */
ExxxReprograming::ErrorCode ExxxReprograming::Open() {
  boost::system::error_code error(boost::system::errc::success, boost::system::system_category());
  int port = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (port < 0) {
    error = boost::system::error_code(errno, boost::system::system_category());
  } else {
    fd_ = port;
    // Initialize the device
    // If an error occurs, set errno as the return value and terminate the process

    // Set raw mode
    termios term = { 0 };
    if (tcgetattr(fd_, &term)) {
      error = boost::system::error_code(errno, boost::system::system_category());
    } else {
      if (!is_usb_rs485_) {
        // 8bit
        // stop bit 1
        // no parity
        // no modem control
        // enable receiving characters0
        term.c_iflag = IGNPAR;
        term.c_cflag = baudrate_ | CS8 | CLOCAL | CREAD;
        term.c_oflag = OPOST;
        term.c_lflag = 0;
        term.c_cc[VTIME] = 0;
        term.c_cc[VMIN] = 1;
        if (tcsetattr(fd_, TCSANOW, &term)) {
          error = boost::system::error_code(errno, boost::system::system_category());
        }

        // set low latency
        serial_struct serial = { 0 };
        if (error.value() == boost::system::errc::success) {
          if (ioctl(fd_, TIOCGSERIAL, &serial)) {
            error = boost::system::error_code(errno, boost::system::system_category());
          }
        }

        if (error.value() == boost::system::errc::success) {
          serial.flags |= ASYNC_LOW_LATENCY;
          if (ioctl(fd_, TIOCSSERIAL, &serial)) {
            error = boost::system::error_code(errno, boost::system::system_category());
          }
        }

        // Flushing port
        if (error.value() == boost::system::errc::success) {
          if (tcflush(fd_, TCIOFLUSH)) {
            error = boost::system::error_code(errno, boost::system::system_category());
          }
        }

        // Set RS485 Half-Duplex mode
        int value = 0;
        if (error.value() == boost::system::errc::success) {
          if (ioctl(fd_, tmc_exxx_servo_motor_protocol::kCti485GetMode, &value) == -1) {
            error = boost::system::error_code(errno, boost::system::system_category());
          }
        }
        if (error.value() == boost::system::errc::success) {
          if (value != tmc_exxx_servo_motor_protocol::kCti485HalfDuplexMode) {
            value = tmc_exxx_servo_motor_protocol::kCti485HalfDuplexMode;
            if (ioctl(fd_, tmc_exxx_servo_motor_protocol::kCti485SetMode, &value) < 0) {
              error = boost::system::error_code(errno, boost::system::system_category());
            }
          }
        }
      } else {
        cfmakeraw(&term);
        if (cfsetispeed(&term, baudrate_)) {
          error = boost::system::error_code(errno, boost::system::system_category());
        }
        if (error.value() == boost::system::errc::success) {
          if (tcsetattr(fd_, TCSANOW, &term)) {
            error = boost::system::error_code(errno, boost::system::system_category());
          }
        }
      }
    }
  }
  return error;
}

/**
 * @brief PROP amp boot section startup process
 *
 * Send the axis number and start the PROP amp boot section.
 * Normally called after PROP amp reset during repro.
 * In forced repro, call while the PROP amp is powered off.
 *
 * Communication for startup process
 *   Host -  PROP amp
 *  Axis number  -> 
 *  Axis number  -> 
 *  Axis number  -> 
 *  Axis number  -> 
 *  Axis number  -> 
 *         <-  \r\n====< Bootloader Ver.1.0.5 by TMC >====
 *         <-  \r\n -- for No_Axis number -- \r\n
 *         <-  \r\n [w]:UPLOAD [g]:BOOT [e]:ERASE [x]:RESET [?]:MENU\r\n
 *         <-  >
 *
 * @param[in]  bootloader_version Bootloader version
 * @param[in]  id                 Axis number of the PROP amp boot section
 * @param[in]  boot_timeout       PROP amp startup timeout time [ns]
 * @return
 * boost::system::errc::success  PROP amp boot section startup successful
 * boost::system::errc::timed_out  Transmission timeout
 * boost::system::errc::no_message  Timeout without receiving PROP amp boot section startup message
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * boost::system::errc::invalid_argument  Failed to wait for the specified time
 * Return errors that occur with poll, read, write
 */
ExxxReprograming::ErrorCode ExxxReprograming::WaitBoot(const int32_t bootloader_version,
                                                       const uint8_t id,
                                                       const int64_t boot_timeout) {
  std::string axis_number = "No_";
  std::string boot_version = "Ver.";
  std::string receive_message;
  std::string boot_message;
  uint32_t output_count = 0;
  int64_t boot_start = Now();
  int64_t start = boot_start;
  int64_t elapsed = boot_start;
  uint8_t receive_data;
  bool is_boot_message = false;
  bool is_axis_number = false;
  boost::system::error_code error;

  axis_number.push_back(id);
  while (true) {
    error = SendData(&id, 1);
    if (error.value() != boost::system::errc::success) {
      break;
    }
    while (true) {
      // Confirm reception of startup message
      error = ReceiveLine(receive_message);
      if (error.value() != boost::system::errc::success) {
        break;
      } else {
        // Check if the startup message is in the received data
        // If the startup message is received, the boot section startup is successful
        boot_message = boot_message + receive_message;
        is_boot_message = (boot_message.find(kBootMessage) != std::string::npos);
        is_axis_number = (boot_message.find(axis_number) != std::string::npos);

        if (is_boot_message && is_axis_number) {
          // After receiving the startup message, confirm the bootloader version.
          boot_version.push_back(bootloader_version + '0');
          if (boot_message.find(boot_version) == std::string::npos) {
            error = boost::system::error_code(boost::system::errc::not_supported, boost::system::system_category());
            break;
          }
          // The PROP amp boot section has a waiting time from receiving the axis number to sending the startup message.
          // During the waiting time, the axis number is sent, and the boot section responds to the received axis number during the waiting time.
          // Received data other than the startup message is unnecessary for subsequent repro processing, so discard the received data.
          while (true) {
            error = ReceiveByte(receive_data);
            if (error.value() != boost::system::errc::success) {
              break;
            }
          }
          // If the discard is completed successfully, the error becomes timed_out
          // If the error is timed_out, set the return value to success
          if (error.value() == boost::system::errc::timed_out) {
            error = boost::system::error_code(boost::system::errc::success, boost::system::system_category());
          }
          break;
        }
      }
    }
    if (is_boot_message && is_axis_number) {
      // If the startup message is received, terminate the process
      break;
    }
    if ((error.value() != boost::system::errc::success) &&
        (error.value() != boost::system::errc::timed_out)) {
      // Terminate the process due to abnormal detection
      break;
    } else {
      elapsed = Now();
      // Output . every second
      if (((elapsed - start) >= kReproIntervalTime) && (output_count < (boot_timeout / kReproIntervalTime))) {
        std::cout << "." << std::flush;
        start = elapsed;
        output_count++;
      }
      if ((elapsed - boot_start) >= (boot_timeout + kBootStartUpWaitTime)) {
        // If the startup message is not received within the timeout period, set the return value to no_message and terminate the process
        error = boost::system::error_code(boost::system::errc::no_message, boost::system::system_category());
        break;
      }
      if (elapsed < start) {
        // Overflow
        error = boost::system::error_code(boost::system::errc::value_too_large, boost::system::system_category());
        break;
      }
    }
    error = WaitNanoSec(kCommandIntervalTime);
    if (error.value() != boost::system::errc::success) {
      break;
    }
  }
  return error;
}

/**
 * @brief PROP amp erase process
 *
 * Erase the firmware of the PROP amp.
 *
 * Communication for erase process
 *   Host -  PROP amp
 *       e  -> 
 *         <-  ERASE ALL FLASH ARE YOU SURE? (Y/N) 
 *       y  -> 
 *         <-  y\r\n
 *         <-  .
 *         <-  .
 *         <-  SUCCESS
 *
 * @return
 * boost::system::errc::success  PROP amp erase successful
 * boost::system::errc::timed_out  Erase process timeout
 * boost::system::errc::no_message  Timeout without receiving erase command confirmation message
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * boost::system::errc::invalid_argument  Failed to wait for the specified time
 * Return errors that occur with poll, read, write
 */
ExxxReprograming::ErrorCode ExxxReprograming::Erase() {
  int64_t start;
  int64_t elapsed;
  boost::system::error_code error;

  error = SendCommand(kEraseCommand, kEraseCheckMessage);
  if (error.value() == boost::system::errc::timed_out) {
    // If there is no response to the erase command, set the return value to no_message
    error = boost::system::error_code(boost::system::errc::no_message, boost::system::system_category());
  }
  if (error.value() == boost::system::errc::success) {
    error = SendData(&kExecuteCommand, 1);
  }
  if (error.value() == boost::system::errc::success) {
    start = Now();
    elapsed = start;
    // Confirm reception of erase completion message until the erase completion wait time elapses
    while (true) {
      error = CheckReceivedMessage(kEraseFinishMessage);
      if (error.value() == boost::system::errc::success) {
        // Since the erase completion message was received, terminate the process
        break;
      } else if ((error.value() != boost::system::errc::illegal_byte_sequence) &&
                 (error.value() != boost::system::errc::timed_out)) {
        // If errors other than timeout and confirmation message not received occur, terminate the process
        break;
      } else {
        // Continue the process as the erase completion message was not received
      }
      error = WaitNanoSec(kCommandIntervalTime);
      if (error.value() != boost::system::errc::success) {
        break;
      }
      elapsed = Now();
      if ((elapsed - start) >= kEraseWaitTime) {
        // If the erase completion wait time elapses, set the return value to timed_out and terminate the process
        error = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
        break;
      }
      if (elapsed < start) {
        // Overflow
        error = boost::system::error_code(boost::system::errc::value_too_large, boost::system::system_category());
        break;
      }
    }
  }
  return error;
}

/**
 * @brief PROP amp firmware write process
 * 
 * Send firmware write data to the PROP amp and rewrite the firmware.
 * Firmware write data is sent using the XMODEM protocol.
 * The corresponding type of XMODEM is XMODEM/SUM.
 * 
 * Communication for firmware write process
 *   Host -  PROP amp
 *       w  -> 
 *         <-  START UPLOAD...\r\n
 *         <-  ARE YOU SURE? (Y/N) 
 *       y  -> 
 *         <-  y\r\n
 *
 *     (XMODEM communication)
 *
 *         <-  <SUCCESS>\r\n
 *
 * @param[in]  flush_data  Firmware write data
 * @return
 * boost::system::errc::success  PROP amp firmware write successful
 * boost::system::errc::timed_out  Communication timeout between PROP amp
 * boost::system::errc::no_message  Timeout without receiving flash write command confirmation message
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * boost::system::errc::invalid_argument  Failed to wait for the specified time
 * Return errors that occur with poll, read, write
 */
ExxxReprograming::ErrorCode ExxxReprograming::Flush(const std::string& flush_data) {
  boost::system::error_code error;
  uint8_t receive_data;

  error = SendCommand(kFlushCommand, kFlushCheckMessage);
  if (error.value() == boost::system::errc::timed_out) {
    // If there is no response to the flash write command, set the return value to no_message
    error = boost::system::error_code(boost::system::errc::no_message, boost::system::system_category());
  } else if (error.value() != boost::system::errc::success) {
    // If errors other than timeout occur, terminate the process
  } else {
    // Since the response to the flash write command consists of multiple lines, discard unnecessary received data
    while (true) {
      error = ReceiveByte(receive_data);
      if (error.value() != boost::system::errc::success) {
        break;
      }
    }
    // If the discard is completed successfully, the error becomes timed_out
    // If the error is timed_out, send the command
    if (error.value() == boost::system::errc::timed_out) {
      error = SendData(&kExecuteCommand, 1);
    }
  }
  if (error.value() == boost::system::errc::success) {
    error = FlushXmodem(flush_data);
  }
  return error;
}

/**
 * @brief PROP amp firmware execution process
 *
 * Terminate the PROP amp boot section and execute the firmware.
 *
 * Communication for firmware execution process
 *   Host -  PROP amp
 *       g  -> 
 *         <-  GO !!\r\n
 *
 * @param   void
 * @return
 * boost::system::errc::success  PROP amp firmware execution successful
 * boost::system::errc::no_message  Timeout without receiving firmware start command response message
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * boost::system::errc::invalid_argument  Failed to wait for the specified time
 * Return errors that occur with write
 */
ExxxReprograming::ErrorCode ExxxReprograming::Run() {
  boost::system::error_code error;

  error = SendCommand(kRunCommand, kRunCheckMessage);
  if (error.value() == boost::system::errc::timed_out) {
    // If there is no response to the firmware start command, set the return value to no_message
    error = boost::system::error_code(boost::system::errc::no_message, boost::system::system_category());
  }
  return error;
}

/**
 * @brief Firmware write process using XMODEM communication
 * 
 * Send firmware write data using the XMODEM protocol.
 * The corresponding type of XMODEM is XMODEM/SUM.
 * 
 * @param[in]  flush_data  Firmware write data
 * @return
 * boost::system::errc::success  PROP amp firmware write successful
 * boost::system::errc::timed_out  Communication timeout between PROP amp, XMODEM communication interrupted
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * boost::system::errc::invalid_argument  Failed to wait for the specified time
 * Return errors that occur with poll, read, write
 */
ExxxReprograming::ErrorCode ExxxReprograming::FlushXmodem(const std::string& flush_data) {
  boost::system::error_code error;
  uint32_t retry_count = 0;
  uint32_t send_pos = 0;
  uint32_t output_count = 0;
  uint32_t flush_ratio = 0;
  uint8_t block_number = 1;
  uint8_t receive_data;
  bool is_flush_xmodem = true;

  // Wait for NAK (transmission request) reception
  while (true) {
    error = ReceiveByte(receive_data);
    if ((error.value() == boost::system::errc::success) && (receive_data == kXmodemNAK)) {
      break;
    } else if ((error.value() != boost::system::errc::success) &&
               (error.value() != boost::system::errc::timed_out)) {
      // If errors other than timeout occur, terminate the process
      break;
    } else {
      // Continue the process as NAK was not received
    }
    retry_count++;
    if (retry_count >= kFlushXmodemMaxRetryCount) {
      // If the retry count exceeds the specified number, set the return value to timed_out and terminate the process
      error = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
      break;
    }
  }
  // Send the first block
  if (error.value() == boost::system::errc::success) {
    error = SendFlushBlockData(flush_data, send_pos, block_number);
  }
  if (error.value() == boost::system::errc::success) {
    error = WaitNanoSec(kFlushIntervalTime);
  }
  if (error.value() == boost::system::errc::success) {
    retry_count = 0;
    // Send firmware write data using XMODEM communication
    // If there is no received data or NAK is received 10 times in a row, set the return value to timed_out and terminate the process
    // If CAN (cancel) is received, set the return value to timed_out and terminate the process (do not retry)
    while (is_flush_xmodem) {
      receive_data = 0;
      error = ReceiveByte(receive_data);
      if (error.value() == boost::system::errc::timed_out) {
        retry_count++;
      } else if (error.value() != boost::system::errc::success) {
        break;
      } else {
        switch (receive_data) {
          case kXmodemACK:
            // Received a normal response from PROP. Send the next data
            retry_count = 0;
            send_pos += kXmodemDataSize;
            block_number++;
            // Output . every 10% of writing
            flush_ratio = ((send_pos * 10) / flush_data.length());
            if (flush_ratio > 10) {
              flush_ratio = 10;
            }
            for (uint32_t i = output_count; i < flush_ratio; ++i) {
              std::cout << "." << std::flush;
            }
            output_count = flush_ratio;
            if (send_pos >= flush_data.length()) {
              // Since the write data transmission is complete, send EOT (transfer end)
              // If the PROP amp firmware still has remaining write data internally, it will write and respond.
              // If there is no remaining write data, it will respond immediately.
              error = SendCommand(kXmodemEOT, kFlushFinishMessage);
              is_flush_xmodem = false;
            } else {
              error = SendFlushBlockData(flush_data, send_pos, block_number);
              if (error.value() != boost::system::errc::success) {
                is_flush_xmodem = false;
              }
            }
            break;
          case kXmodemNAK:
            // Resend
            retry_count++;
            if (retry_count < kFlushXmodemMaxRetryCount) {
              error = SendFlushBlockData(flush_data, send_pos, block_number);
              if (error.value() != boost::system::errc::success) {
                is_flush_xmodem = false;
              }
            }
            break;
          case kXmodemCAN:
            // Cancel. Set the return value to timed_out and terminate the process
            error = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
            is_flush_xmodem = false;
            break;
          default :
            retry_count++;
            break;
        }
      }
      if (retry_count >= kFlushXmodemMaxRetryCount) {
        // If the retry count exceeds the specified number, set the return value to timed_out and terminate the process
        error = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
        break;
      }
      // Guard with is_flush_xmodem to prevent error from being overwritten after XMODEM communication ends
      if (is_flush_xmodem) {
        error = WaitNanoSec(kFlushIntervalTime);
        if (error.value() != boost::system::errc::success) {
          break;
        }
      }
    }
  }
  return error;
}

/**
 * @brief Send a command and confirm the reception of a confirmation message
 *
 * @param[in]  command        Command to send
 * @param[in]  check_message  Confirmation message
 * @return
 * boost::system::errc::success  Confirmation message received
 * boost::system::errc::timed_out  Command transmission timeout and message reception timeout
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * boost::system::errc::invalid_argument  Failed to wait for the specified time
 * Return errors that occur with write
 */
ExxxReprograming::ErrorCode ExxxReprograming::SendCommand(const uint8_t command, const std::string& check_message) {
  uint32_t retry_count = 0;
  boost::system::error_code error;

  while (true) {
    error = SendData(&command, 1);
    if (error.value() != boost::system::errc::success) {
      break;
    }
    error = CheckReceivedMessage(check_message);
    if (error.value() == boost::system::errc::success) {
      // Since the confirmation message was received, terminate the process
      break;
    } else if ((error.value() != boost::system::errc::timed_out) &&
               (error.value() != boost::system::errc::illegal_byte_sequence)) {
      // If errors other than timeout and confirmation message not received occur, do not retry and terminate the process
      break;
    } else {
      retry_count++;
      if (retry_count >= kSendCommandMaxRetryCount) {
        // If the retry count exceeds the specified number, set the return value to timed_out and terminate the process
        error = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
        break;
      }
    }
    error = WaitNanoSec(kCommandIntervalTime);
    if (error.value() != boost::system::errc::success) {
      break;
    }
  }
  return error;
}

/**
 * @brief Confirm the reception of a confirmation message
 *
 * @param[in]  check_message  Confirmation message
 * @return
 * boost::system::errc::success  Confirmation message received
 * boost::system::errc::illegal_byte_sequence  Confirmation message not received
 * boost::system::errc::timed_out  Message reception timeout
 * boost::system::errc::value_too_large  Overflow of variable for reception timeout measurement
 * Return errors that occur with poll, read
 */
ExxxReprograming::ErrorCode ExxxReprograming::CheckReceivedMessage(const std::string& check_message) {
  std::string receive_message;
  boost::system::error_code error;

  error = ReceiveLine(receive_message);
  if (error.value() == boost::system::errc::success) {
    // If one line is successfully received, check if the confirmation message is in the received data
    // If the confirmation message is not in the received data, set the return value to illegal_byte_sequence
    if (receive_message.find(check_message) == std::string::npos) {
      error = boost::system::error_code(boost::system::errc::illegal_byte_sequence, boost::system::system_category());
    }
  }
  return error;
}

/**
 * @brief XMODEM communication firmware write data transmission
 *
 * @param[in]  flush_data     Firmware write data
 * @param[in]  send_pos       Write start position
 * @param[in]  block_number   Block number
 * @return
 * boost::system::errc::success  Write data transmission successful
 * boost::system::errc::timed_out  Write data transmission timeout
 * boost::system::errc::value_too_large  Overflow of variable for write data transmission timeout measurement
 * boost::system::errc::invalid_argument  Failed to wait for the specified time
 * Return errors that occur with write
 */
ExxxReprograming::ErrorCode ExxxReprograming::SendFlushBlockData(const std::string& flush_data,
                                                                 uint32_t send_pos,
                                                                 uint8_t block_number) {
  boost::system::error_code error;
  uint32_t data_size = kXmodemDataSize;
  uint8_t check_sum = 0;
  std::array<uint8_t, kXmodemSendSize> send_buffer;

  send_buffer[kXmodemHeaderPos]          = kXmodemSOH;
  send_buffer[kXmodemBlockNumberPos]     = block_number;
  send_buffer[kXmodemBlockNumberCompPos] = (~block_number & 0xFF);
  // Check if the remaining data is less than 128
  if ((flush_data.length() - send_pos) < kXmodemDataSize) {
    data_size = flush_data.length() - send_pos;
  }
  for (uint32_t i = 0; i < data_size; ++i) {
    send_buffer[kXmodemHeaderSize + i] = flush_data[send_pos + i];
  }
  // If the write data is less than 128 bytes, fill the remainder with EOF
  for (uint32_t i = data_size; i < kXmodemDataSize; ++i) {
    send_buffer[kXmodemHeaderSize + i] = kXmodemEOF;
  }
  // Checksum calculation Add data part
  for (uint32_t i = 0; i < kXmodemDataSize; ++i) {
    // Checksum is the two's complement of the lower 8 bits
    check_sum += send_buffer[kXmodemHeaderSize + i];
  }
  send_buffer[kXmodemCheckSumPos] = check_sum;
  error = SendData(&send_buffer[0], kXmodemSendSize);
  return error;
}

/**
 * @brief Transmit specified bytes
 *
 * @param[in]  data        Transmission buffer
 * @param[in]  data_bytes  Number of bytes to transmit
 * @return
 * boost::system::errc::success  Successfully transmitted the specified number of bytes
 * boost::system::errc::timed_out  Failed to transmit the specified number of bytes within the timeout
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * boost::system::errc::invalid_argument  Failed to wait for the specified time
 * Return errors that occur with write
 */
ExxxReprograming::ErrorCode ExxxReprograming::SendData(const uint8_t* data, uint32_t data_bytes) {
  boost::system::error_code error;
  int64_t start = Now();
  int64_t elapsed = start;
  uint32_t num_done = 0;

  while ((elapsed - start) < timeout_) {
    int32_t result = write(fd_, &data[num_done], data_bytes - num_done);
    if (result < 0) {
      if (errno != EAGAIN) {
        error = boost::system::error_code(errno, boost::system::system_category());
        break;
      } else {
        // If the return value is EAGAIN, wait for the specified time and continue the transmission process
        error = WaitNanoSec(sleep_tick_);
        if (error.value() != boost::system::errc::success) {
          break;
        }
      }
    } else {
      num_done += result;
      if (num_done == data_bytes) {
        // If the number of transmitted bytes equals the argument data_bytes, set the return value to success and terminate the process
        error = boost::system::error_code(boost::system::errc::success, boost::system::system_category());
        break;
      } else if (num_done > data_bytes) {
        RCLCPP_FATAL(rclcpp::get_logger("exxx_reprograming"), "NOT REACHED");
      } else {
        // Since the number of transmitted bytes has not reached the argument data_bytes, continue the transmission process
      }
    }
    int64_t last_elapsed = elapsed;
    elapsed = Now();
    if (elapsed < last_elapsed) {
      // Overflow
      error = boost::system::error_code(boost::system::errc::value_too_large, boost::system::system_category());
      break;
    }
  }
  if (num_done != data_bytes) {
    // If the number of bytes transmitted is less than the argument data_bytes, set the return value to timed_out
    error = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
  }
  return error;
}

/**
 * @brief Receive until delimiter
 *
 * Receive until '\n' or '?'
 *
 * @param[out] receive_line  Received data
 * @return
 * boost::system::errc::success  Successfully received until delimiter
 * boost::system::errc::timed_out  Timeout without receiving delimiter (received data is set in receive_line)
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * Return errors that occur with poll, read
 */
ExxxReprograming::ErrorCode ExxxReprograming::ReceiveLine(std::string& receive_line) {
  boost::system::error_code error;
  std::string receive_message;
  uint8_t receive_data;
  int64_t start = Now();
  int64_t elapsed = start;
  uint32_t recv_size = 0;

  while ((elapsed - start) < timeout_) {
    // Receive 1 byte
    error = ReceiveByte(receive_data);
    if (error.value() == boost::system::errc::timed_out) {
      // Continue the process in case of timeout
    } else if (error.value() != boost::system::errc::success) {
      // If the error is not success or timed_out, return the occurred error
      break;
    } else {
      receive_message.push_back(receive_data);
      recv_size++;
      if ((receive_data == kReceiveLineDelimiter1) || (receive_data == kReceiveLineDelimiter2)) {
        // If the delimiter is received, set the received data in the argument receive_line and set the return value to success
        receive_line = receive_message;
        error = boost::system::error_code(boost::system::errc::success, boost::system::system_category());
        break;
      }
    }
    int64_t last_elapsed = elapsed;
    elapsed = Now();
    if (elapsed < last_elapsed) {
      // Overflow
      error = boost::system::error_code(boost::system::errc::value_too_large, boost::system::system_category());
      break;
    }
  }
  if ((elapsed - start) >= timeout_) {
    if (recv_size != 0) {
      // If data was received during timeout, set the received data in the argument receive_line and set the return value to success
      error = boost::system::error_code(boost::system::errc::success, boost::system::system_category());
      receive_line = receive_message;
    }
  }
  return error;
}

/**
 * @brief Receive 1 byte
 *
 * @param[out] data_out    Received data
 * @return
 * boost::system::errc::success  Successfully received 1 byte
 * boost::system::errc::timed_out  Reception timeout
 * Return errors that occur with poll, read
 */
ExxxReprograming::ErrorCode ExxxReprograming::ReceiveByte(uint8_t& data_out) {
  boost::system::error_code error;
  uint8_t receive_data;
  struct pollfd poll_fd[1];
  struct timespec poll_timeout;
  poll_fd[0].fd = fd_;
  poll_fd[0].events = POLLIN | POLLPRI;
  int64_t remain_timeout = timeout_;
  poll_timeout.tv_sec = remain_timeout / 1000000000LL;
  poll_timeout.tv_nsec = remain_timeout % 1000000000LL;
  int ready = ppoll(&poll_fd[0], 1, &poll_timeout, NULL);
  if (ready == 0) {
    // timeout
    error = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
  } else if (ready < 0) {
    error = boost::system::error_code(errno, boost::system::system_category());
  } else {
    int32_t result = read(fd_, &receive_data, 1);
    if (result < 0) {
      error = boost::system::error_code(errno, boost::system::system_category());
    } else {
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("exxx_reprograming"),
                            "Read:" << std::hex  << std::setw(2) << std::setfill('0')
                            << static_cast<int32_t>(receive_data));
      }
      data_out = receive_data;
      error = boost::system::error_code(boost::system::errc::success, boost::system::system_category());
    }
  }
  return error;
}

}  // namespace tmc_exxx_servo_motor_protocol
