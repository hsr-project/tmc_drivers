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
const uint32_t kSendCommandMaxRetryCount      = 10;                 /// Maximum number of retries for sending a command
const uint32_t kFlushXmodemMaxRetryCount      = 100;                /// Maximum number of retries for receiving XMODEM communication
const uint8_t  kEraseCommand                  = 0x65;               /// Erase command 'e'
const uint8_t  kFlushCommand                  = 0x77;               /// Firmware write command 'w'
const uint8_t  kRunCommand                    = 0x67;               /// Firmware start command 'g'
const uint8_t  kExecuteCommand                = 0x79;               /// Execute command 'y'
const char*    kBootMessage                   = "Bootloader";       /// Startup message for PROP amplifier boot section
const char*    kEraseCheckMessage             = "ERASE ALL FLASH";  /// Erase confirmation message
const char*    kEraseFinishMessage            = "COMPLETED";        /// Erase completion message
const char*    kFlushCheckMessage             = "START UPLOAD";     /// Firmware write confirmation message
const char*    kFlushFinishMessage            = "SUCCESS";          /// Firmware write completion message
const char*    kRunCheckMessage               = "GO";               /// Firmware start message
const uint8_t  kXmodemSOH                     = 0x01;               /// XMODEM communication block start
const uint8_t  kXmodemEOT                     = 0x04;               /// XMODEM communication transfer end
const uint8_t  kXmodemACK                     = 0x06;               /// XMODEM communication normal response
const uint8_t  kXmodemNAK                     = 0x15;               /// XMODEM communication transmission request and negative response
const uint8_t  kXmodemCAN                     = 0x18;               /// XMODEM communication cancellation
const uint8_t  kXmodemEOF                     = 0xFF;               /// XMODEM communication padding
const uint32_t kXmodemSendSize                = 132;                /// XMODEM communication send data size
const uint32_t kXmodemDataSize                = 128;                /// XMODEM communication data size
const uint32_t kXmodemHeaderSize              = 3;                  /// XMODEM communication header + block number size
const uint32_t kXmodemHeaderPos               = 0;                  /// XMODEM communication send data header position
const uint32_t kXmodemBlockNumberPos          = 1;                  /// XMODEM communication send data block number position
const uint32_t kXmodemBlockNumberCompPos      = 2;                  /// XMODEM communication send data block number complement position
const uint32_t kXmodemCheckSumPos             = 131;                /// XMODEM communication send data checksum position
const int64_t  kBootStartUpWaitTime           = 3000000000;         /// Waiting time for PROP amplifier boot section startup [ns]
const int64_t  kReproIntervalTime             = 1000000000;         /// Output interval of '.' during PROP amplifier startup wait [ns]
const int64_t  kCommandIntervalTime           = 50000000;           /// Command sending interval [ns]
const int64_t  kEraseWaitTime                 = 1000000000;         /// Waiting time for erase completion [ns]
const int64_t  kFlushIntervalTime             = 10000000;           /// Firmware write XMODEM communication receiving interval [ns]
const uint8_t  kReceiveLineDelimiter1         = 0x0A;               /// '\n'(line feed)
const uint8_t  kReceiveLineDelimiter2         = 0x3F;               /// '?'

/**
 * @brief Wait for specified time [ns]
 *
 * @param[in]  nsec         Waiting time [ns]
 * @return
 * boost::system::errc::success  Waiting for specified time successful
 * boost::system::errc::invalid_argument  Waiting for specified time failed
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
      // Both occur due to arguments of the clock_nanosleep function, so invalid_argument is set as the return value
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
 * @param[in]   is_usb_rs485  USB usage (true: Using USB, false: Not using USB)
 * @param[in]   timeout       Timeout time for send/receive [ns]
 * @param[in]   sleep_tick    Retry wait time [ns] when the return value of write function is EAGIN
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
 * @brief Open communication device
 *
 * @return
 * boost::system::errc::success  Device open successful
 * Return errors occurring with open, tcgetattr, tcsetattr, ioctl, tcflush, cfsetispeed, tcsetattr
 */
ExxxReprograming::ErrorCode ExxxReprograming::Open() {
  boost::system::error_code error(boost::system::errc::success, boost::system::system_category());
  int port = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (port < 0) {
    error = boost::system::error_code(errno, boost::system::system_category());
  } else {
    fd_ = port;
    // Initialize the device
    // If an error occurs, set errno as the return value and terminate the processing

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
 * @brief PROP amplifier boot section startup processing
 *
 * Send axis number to start the boot section of the PROP amplifier.
 * Normally called after PROP amplifier reset during repro.
 * Called while the PROP amplifier is powered off during forced repro.
 *
 * Startup processing communication
 *   Host -  PROP amplifier
 *  Axis number  -> 
 *  Axis number  -> 
 *  Axis number  -> 
 *  Axis number  -> 
 *  Axis number  -> 
 *         <-  \r\n====< Bootloader Ver.1.0.5 by TMC >====
 *         <-  \r\n -- for No_axis_number -- \r\n
 *         <-  \r\n [w]:UPLOAD [g]:BOOT [e]:ERASE [x]:RESET [?]:MENU\r\n
 *         <-  >
 *
 * @param[in]  bootloader_version Bootloader version
 * @param[in]  id                 Axis number of the PROP amplifier boot section
 * @param[in]  boot_timeout       Startup timeout time of PROP amplifier [ns]
 * @return
 * boost::system::errc::success  PROP amplifier boot section startup successful
 * boost::system::errc::timed_out  Sending timeout
 * boost::system::errc::no_message  Timeout with no receipt of PROP amplifier boot section startup message
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * boost::system::errc::invalid_argument  Waiting for specified time failed
 * Return errors occurring with poll, read, write
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
      // Receive confirmation for startup message
      error = ReceiveLine(receive_message);
      if (error.value() != boost::system::errc::success) {
        break;
      } else {
        // Check whether startup message is within the received data
        // If startup message is received, boot section startup successful
        boot_message = boot_message + receive_message;
        is_boot_message = (boot_message.find(kBootMessage) != std::string::npos);
        is_axis_number = (boot_message.find(axis_number) != std::string::npos);

        if (is_boot_message && is_axis_number) {
          // After receiving startup message, confirm bootloader version.
          boot_version.push_back(bootloader_version + '0');
          if (boot_message.find(boot_version) == std::string::npos) {
            error = boost::system::error_code(boost::system::errc::not_supported, boost::system::system_category());
            break;
          }
          // PROP amplifier boot section has waiting time until sending startup message after receiving axis number.
          // During wait time, axis number is sent and boot section returns response for received axis number.
          // Received data other than startup message is unnecessary for subsequent repro processing, so discard received data.
          while (true) {
            error = ReceiveByte(receive_data);
            if (error.value() != boost::system::errc::success) {
              break;
            }
          }
          // If discard ends successfully, error becomes timed_out
          // If error is timed_out, set success as the return value
          if (error.value() == boost::system::errc::timed_out) {
            error = boost::system::error_code(boost::system::errc::success, boost::system::system_category());
          }
          break;
        }
      }
    }
    if (is_boot_message && is_axis_number) {
      // Terminate processing if startup message is received
      break;
    }
    if ((error.value() != boost::system::errc::success) &&
        (error.value() != boost::system::errc::timed_out)) {
      // Terminate processing for anomaly detection
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
        // If startup message is not received within timeout time, set no_message as the return value and terminate processing
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
 * @brief PROP amplifier erase processing
 *
 * Erase the firmware of PROP amplifier.
 *
 * Erase processing communication
 *   Host -  PROP amplifier
 *       e  -> 
 *         <-  ERASE ALL FLASH ARE YOU SURE? (Y/N) 
 *       y  -> 
 *         <-  y\r\n
 *         <-  .
 *         <-  .
 *         <-  SUCCESS
 *
 * @return
 * boost::system::errc::success  PROP amplifier erase successful
 * boost::system::errc::timed_out  Erase processing timeout
 * boost::system::errc::no_message  Timeout with no receipt of erase command confirmation message
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * boost::system::errc::invalid_argument  Waiting for specified time failed
 * Return errors occurring with poll, read, write
 */
ExxxReprograming::ErrorCode ExxxReprograming::Erase() {
  int64_t start;
  int64_t elapsed;
  boost::system::error_code error;

  error = SendCommand(kEraseCommand, kEraseCheckMessage);
  if (error.value() == boost::system::errc::timed_out) {
    // If there is no response to erase command, set no_message as the return value
    error = boost::system::error_code(boost::system::errc::no_message, boost::system::system_category());
  }
  if (error.value() == boost::system::errc::success) {
    error = SendData(&kExecuteCommand, 1);
  }
  if (error.value() == boost::system::errc::success) {
    start = Now();
    elapsed = start;
    // Until erase completion waiting time elapses, confirm receipt of erase completion message
    while (true) {
      error = CheckReceivedMessage(kEraseFinishMessage);
      if (error.value() == boost::system::errc::success) {
        // Since erase completion message is received, terminate processing
        break;
      } else if ((error.value() != boost::system::errc::illegal_byte_sequence) &&
                 (error.value() != boost::system::errc::timed_out)) {
        // Terminate processing if errors other than timeout and unreceived confirmation message occur
        break;
      } else {
        // Continue processing since erase completion message is not received
      }
      error = WaitNanoSec(kCommandIntervalTime);
      if (error.value() != boost::system::errc::success) {
        break;
      }
      elapsed = Now();
      if ((elapsed - start) >= kEraseWaitTime) {
        // If erase completion waiting time elapses, set timed_out as the return value and terminate processing
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
 * @brief PROP amplifier firmware write processing
 * 
 * Send firmware write data to PROP amplifier and perform firmware rewrite.
 * Firmware write data transmission is performed using XMODEM protocol.
 * The type of XMODEM to be used is XMODEM/SUM.
 * 
 * Firmware write processing communication
 *   Host -  PROP amplifier
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
 * boost::system::errc::success  PROP amplifier firmware write successful
 * boost::system::errc::timed_out  Communication timeout between PROP amplifier
 * boost::system::errc::no_message  Timeout with no receipt of flash write command confirmation message
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * boost::system::errc::invalid_argument  Waiting for specified time failed
 * Return errors occurring with poll, read, write
 */
ExxxReprograming::ErrorCode ExxxReprograming::Flush(const std::string& flush_data) {
  boost::system::error_code error;
  uint8_t receive_data;

  error = SendCommand(kFlushCommand, kFlushCheckMessage);
  if (error.value() == boost::system::errc::timed_out) {
    // If there is no response to flash write command, set no_message as the return value
    error = boost::system::error_code(boost::system::errc::no_message, boost::system::system_category());
  } else if (error.value() != boost::system::errc::success) {
    // Terminate processing if errors other than timeout occur
  } else {
    // Since response to flash write command is multiple lines, discard unnecessary received data
    while (true) {
      error = ReceiveByte(receive_data);
      if (error.value() != boost::system::errc::success) {
        break;
      }
    }
    // If discard ends successfully, error becomes timed_out
    // If error is timed_out, perform command sending
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
 * @brief PROP amplifier firmware execution processing
 *
 * Terminate the boot section of PROP amplifier and execute firmware.
 *
 * Firmware execution processing communication
 *   Host -  PROP amplifier
 *       g  -> 
 *         <-  GO !!\r\n
 *
 * @param   void
 * @return
 * boost::system::errc::success  PROP amplifier firmware execution successful
 * boost::system::errc::no_message  Timeout with no receipt of firmware start command response message
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * boost::system::errc::invalid_argument  Waiting for specified time failed
 * Return errors occurring with write
 */
ExxxReprograming::ErrorCode ExxxReprograming::Run() {
  boost::system::error_code error;

  error = SendCommand(kRunCommand, kRunCheckMessage);
  if (error.value() == boost::system::errc::timed_out) {
    // If there is no response to firmware start command, set no_message as the return value
    error = boost::system::error_code(boost::system::errc::no_message, boost::system::system_category());
  }
  return error;
}

/**
 * @brief Firmware write processing via XMODEM communication
 * 
 * Send firmware write data using XMODEM protocol.
 * The type of XMODEM to be used is XMODEM/SUM.
 * 
 * @param[in]  flush_data  Firmware write data
 * @return
 * boost::system::errc::success  PROP amplifier firmware write successful
 * boost::system::errc::timed_out  Communication timeout between PROP amplifier, XMODEM communication cancellation
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * boost::system::errc::invalid_argument  Waiting for specified time failed
 * Return errors occurring with poll, read, write
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
      // Terminate processing if errors other than timeout occur
      break;
    } else {
      // Continue processing since NAK is not received
    }
    retry_count++;
    if (retry_count >= kFlushXmodemMaxRetryCount) {
      // If retry count exceeds specified number, set timed_out as the return value and terminate processing
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
    // If no received data or NAK is received 10 times in a row, set timed_out as the return value and terminate processing
    // If CAN (cancellation) is received, set timed_out as the return value and terminate processing (do not retry or resend)
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
            // Normal response received from PROP. Send next data
            retry_count = 0;
            send_pos += kXmodemDataSize;
            block_number++;
            // Output . every 10% write
            flush_ratio = ((send_pos * 10) / flush_data.length());
            if (flush_ratio > 10) {
              flush_ratio = 10;
            }
            for (uint32_t i = output_count; i < flush_ratio; ++i) {
              std::cout << "." << std::flush;
            }
            output_count = flush_ratio;
            if (send_pos >= flush_data.length()) {
              // Since send of write data ends, send EOT (transfer end)
              // If PROP amplifier firmware has remaining write data internally, write and return response.
              // If no write data is held, return response immediately.
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
            // Cancellation. Set timed_out as the return value and terminate processing
            error = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
            is_flush_xmodem = false;
            break;
          default :
            retry_count++;
            break;
        }
      }
      if (retry_count >= kFlushXmodemMaxRetryCount) {
        // If retry count exceeds specified number, set timed_out as the return value and terminate processing
        error = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
        break;
      }
      // To prevent error from being rewritten after XMODEM communication ends, guard with is_flush_xmodem
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
 * @brief Send command and confirm receipt of confirmation message
 *
 * @param[in]  command        Send command
 * @param[in]  check_message  Confirmation message
 * @return
 * boost::system::errc::success  Confirmation message received
 * boost::system::errc::timed_out  Command send timeout and message receive timeout
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * boost::system::errc::invalid_argument  Waiting for specified time failed
 * Return errors occurring with write
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
      // Since confirmation message is received, terminate processing
      break;
    } else if ((error.value() != boost::system::errc::timed_out) &&
               (error.value() != boost::system::errc::illegal_byte_sequence)) {
      // Terminate processing without retry in cases other than timeout and unreceived confirmation message
      break;
    } else {
      retry_count++;
      if (retry_count >= kSendCommandMaxRetryCount) {
        // If retry count exceeds specified number, set timed_out as the return value and terminate processing
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
 * @brief Confirm receipt of confirmation message
 *
 * @param[in]  check_message  Confirmation message
 * @return
 * boost::system::errc::success  Confirmation message received
 * boost::system::errc::illegal_byte_sequence  Confirmation message not received
 * boost::system::errc::timed_out  Message receive timeout
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * Return errors occurring with poll, read
 */
ExxxReprograming::ErrorCode ExxxReprograming::CheckReceivedMessage(const std::string& check_message) {
  std::string receive_message;
  boost::system::error_code error;

  error = ReceiveLine(receive_message);
  if (error.value() == boost::system::errc::success) {
    // If one line is received successfully, check whether confirmation message is received
    // If confirmation message is not within received data, set illegal_byte_sequence as the return value
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
 * boost::system::errc::success  Write data send successful
 * boost::system::errc::timed_out  Write data send timeout
 * boost::system::errc::value_too_large  Overflow of variable for write data send timeout measurement
 * boost::system::errc::invalid_argument  Waiting for specified time failed
 * Return errors occurring with write
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
  // Check if remaining data is less than 128
  if ((flush_data.length() - send_pos) < kXmodemDataSize) {
    data_size = flush_data.length() - send_pos;
  }
  for (uint32_t i = 0; i < data_size; ++i) {
    send_buffer[kXmodemHeaderSize + i] = flush_data[send_pos + i];
  }
  // If write data is less than 128 bytes, pad the remaining with EOF
  for (uint32_t i = data_size; i < kXmodemDataSize; ++i) {
    send_buffer[kXmodemHeaderSize + i] = kXmodemEOF;
  }
  // Checksum calculation, summing up data part
  for (uint32_t i = 0; i < kXmodemDataSize; ++i) {
    // Checksum is the lower 8-bit two's complement
    check_sum += send_buffer[kXmodemHeaderSize + i];
  }
  send_buffer[kXmodemCheckSumPos] = check_sum;
  error = SendData(&send_buffer[0], kXmodemSendSize);
  return error;
}

/**
 * @brief Send specified bytes
 *
 * @param[in]  data        Send buffer
 * @param[in]  data_bytes  Number of bytes to send
 * @return
 * boost::system::errc::success  Data of specified byte count send successful
 * boost::system::errc::timed_out  Timeout before sending data of specified byte count
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * boost::system::errc::invalid_argument  Waiting for specified time failed
 * Return errors occurring with write
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
        // If return value is EAGAIN, wait specified time and continue send processing
        error = WaitNanoSec(sleep_tick_);
        if (error.value() != boost::system::errc::success) {
          break;
        }
      }
    } else {
      num_done += result;
      if (num_done == data_bytes) {
        // If number of sent bytes equals argument data_bytes, set success as the return value and terminate processing
        error = boost::system::error_code(boost::system::errc::success, boost::system::system_category());
        break;
      } else if (num_done > data_bytes) {
        RCLCPP_FATAL(rclcpp::get_logger("exxx_reprograming"), "NOT REACHED");
      } else {
        // Continue send processing since number of sent bytes has not reached argument data_bytes
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
    // If unable to send the count specified in argument data_bytes, set timed_out as the return value
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
 * boost::system::errc::success  Received until delimiter successful
 * boost::system::errc::timed_out  Timeout without receiving delimiter (received data set in receive_line)
 * boost::system::errc::value_too_large  Overflow of variable for timeout measurement
 * Return errors occurring with poll, read
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
      // Continue processing in case of timeout
    } else if (error.value() != boost::system::errc::success) {
      // Return the occurred error in case other than success or timed_out
      break;
    } else {
      receive_message.push_back(receive_data);
      recv_size++;
      if ((receive_data == kReceiveLineDelimiter1) || (receive_data == kReceiveLineDelimiter2)) {
        // If delimiter is received, set received data in argument receive_line and set success as the return value
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
      // If data is received during timeout, set received data in argument receive_line and set success as the return value
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
 * boost::system::errc::success  1 byte receive successful
 * boost::system::errc::timed_out  Receive timeout
 * Return errors occurring with poll, read
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
