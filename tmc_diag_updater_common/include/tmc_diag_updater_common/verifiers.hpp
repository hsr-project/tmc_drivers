/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
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
#ifndef TMC_DIAG_UPDATER_COMMON_TMC_DIAG_UPDATER_COMMON_VERIVIERS_HPP_
#define TMC_DIAG_UPDATER_COMMON_TMC_DIAG_UPDATER_COMMON_VERIVIERS_HPP_

#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// Placed here due to lint issues, planned to be addressed in the 2020.04 release
#include <array>

#include <rclcpp/rclcpp.hpp>

using diagnostic_msgs::msg::DiagnosticStatus;

namespace tmc_diag_updater_common {

// Operate the error level and description to be ultimately set in DiagUpdater as a single unit
typedef int DiagErrorLevel;
using Summary = std::pair<DiagErrorLevel, std::string>;
using Summaries = std::vector<Summary>;

// Namespace of the validator
namespace verifier {

static void InitSummary(Summary& summary) {
  summary.first = DiagnosticStatus::OK;
  summary.second = "OK";
}

template <class MsgPtr>
class Interface {
 public:
  using SharedPtr = std::shared_ptr<Interface>;

  explicit Interface(const Summary& summary = Summary(DiagnosticStatus::ERROR, "Has no verified data"))
      : summary_(summary) {}

  virtual ~Interface() = default;

  // Validation interface
  // Returns the updated Summary
  virtual Summary Verify(const MsgPtr& stamped_msg) {
    UpdateSummary(stamped_msg);
    return summary_;
  }

  // Add data to /diagnostics
  // Measured cycle, acceleration, etc.
  virtual void AddValues(diagnostic_updater::DiagnosticStatusWrapper& stat) const = 0;

 protected:
  // Update the state
  // Implement how to validate the topic in derived classes
  virtual void UpdateSummary(const MsgPtr& stamped_msg) = 0;

  // Set OK state
  // The Summary of the OK state should be as unified as possible for clarity
  void SetAsOK() { InitSummary(summary_); }
  // Set arbitrary state
  void SetAs(const DiagErrorLevel id, const std::string& msg) {
    summary_.first = id;
    summary_.second = msg;
  }

  Summary GetSummary() const { return summary_; }

 private:
  // Maintain the latest state
  // If self-validation cannot be performed, return the latest state (such as when passed in a nullptr state)
  Summary summary_;
};


template <class MsgPtr>
class Disconnection : public Interface<MsgPtr> {
 public:
  static typename Interface<MsgPtr>::SharedPtr Create(rclcpp::Node::SharedPtr node) {
    const double timeout_sec = node->declare_parameter<double>("verifiers.disconnection.timeout_sec", 1.0);
    return typename Interface<MsgPtr>::SharedPtr(new Disconnection<MsgPtr>(node, timeout_sec));
  }

  explicit Disconnection(rclcpp::Node::SharedPtr node, double timeout_sec)
      : node_(node),
        latest_clock_stamp_(node_->get_clock()->now()),
        timeout_(rclcpp::Duration::from_seconds(timeout_sec)) {
    if (timeout_ <= rclcpp::Duration::from_seconds(0.0)) {
      throw std::invalid_argument("Timeout should be positive value");
    }
  }

  virtual void UpdateSummary(const MsgPtr& stamped_msg) {
    const rclcpp::Time now = node_->get_clock()->now();
    if (stamped_msg == nullptr) {
      // Interruption judgment
      if (now - latest_clock_stamp_ > timeout_) {
        this->SetAs(DiagnosticStatus::ERROR, "Disconnected");
      }
      return;
    }

    this->SetAsOK();
    latest_clock_stamp_ = now;
    latest_msg_stamp_ = stamped_msg->header.stamp;
  }

  virtual void AddValues(diagnostic_updater::DiagnosticStatusWrapper& stat) const {
    stat.add("Latest message stamp [sec]", RCL_NS_TO_S(latest_msg_stamp_.nanoseconds()));
    stat.add("Latest message stamp [nsec]", RCL_S_TO_NS(latest_msg_stamp_.nanoseconds()));
    Summary summary = this->GetSummary();
    // Has no verified data is set during initialization, so no need to update.
    if (summary.second != "Has no verified data") {
      stat.summaryf(summary.first, summary.second.c_str());
    }
  }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time latest_clock_stamp_;
  rclcpp::Time latest_msg_stamp_;
  const rclcpp::Duration timeout_;
};


// Class to verify whether it is subscribed at the expected cycle
// Has two-stage thresholds to switch between WARN and ERROR
// Additionally, verify whether communication is maintained
//
// Frequency is determined using a queue
// Determine from the difference between the timestamps at the head and tail of the queue
template <class MsgPtr>
class UnexpectedRate : public Interface<MsgPtr> {
 public:
  static typename Interface<MsgPtr>::SharedPtr Create(rclcpp::Node::SharedPtr node) {
    const double warn_hz = node->declare_parameter<double>("verifiers.unexpected_rate.warn_hz", 50.0);
    const double error_hz = node->declare_parameter<double>("verifiers.unexpected_rate.error_hz", 25.0);

    const int window_size = node->declare_parameter<int>("verifiers.unexpected_rate.window_size", 2);
    // Negative values are not anticipated in the constructor, so they are considered errors
    if (window_size < 0) {
      throw std::invalid_argument("Window size for computing rate should be positive value");
    }
    return typename Interface<MsgPtr>::SharedPtr(new UnexpectedRate<MsgPtr>(warn_hz, error_hz, window_size));
  }

  explicit UnexpectedRate(double warn_hz, double error_hz, uint32_t window_size)
      : warn_hz_(warn_hz),
        error_hz_(error_hz),
        actual_hz_(std::numeric_limits<double>::quiet_NaN()),
        window_size_(window_size) {
    if (warn_hz_ < 0.0 || error_hz_ < 0.0) {
      throw std::invalid_argument("Specified rate is invalid. It should be positive value");
    }
    if (warn_hz_ < error_hz_) {
      throw std::invalid_argument("Warning hz should be greater than error hz.");
    }
    // Assume positive values of 2 or more
    // Not an algorithm to determine frequency with 1 data
    if (window_size_ < 2) {
      throw std::invalid_argument("Window size should be greater than 1 for computing rate");
    }
  }

  virtual void UpdateSummary(const MsgPtr& stamped_msg) {
    if (stamped_msg == nullptr) {
      return;
    }

    actual_hz_ = std::numeric_limits<double>::quiet_NaN();

    // Queuing
    // Remove old items when the specified number of elements is reached
    times_queue_.push_back(rclcpp::Time(stamped_msg->header.stamp).seconds());
    if (times_queue_.size() > window_size_) {
      times_queue_.pop_front();
    }

    // If there is not enough data in the queue, do not proceed with further processing
    const int queue_size = static_cast<int>(times_queue_.size());
    if (queue_size != window_size_) {
      return;
    }

    // Determine elapsed time
    // Consider it abnormal if it is too small or a negative value
    const double elapse_time_in_queue = times_queue_.back() - times_queue_.front();
    if (elapse_time_in_queue <= std::numeric_limits<double>::epsilon()) {
      this->SetAs(DiagnosticStatus::ERROR, "The same or past timestamp");
      times_queue_.clear();
      return;
    }

    // Determine by calculating frequency
    actual_hz_ = static_cast<double>(queue_size - 1) / elapse_time_in_queue;
    if (actual_hz_ < error_hz_) {
      this->SetAs(DiagnosticStatus::ERROR, "Significantly low rate");
    } else if (actual_hz_ < warn_hz_) {
      this->SetAs(DiagnosticStatus::WARN, "Slightly low rate");
    } else {
      this->SetAsOK();
    }
  }

  virtual void AddValues(diagnostic_updater::DiagnosticStatusWrapper& stat) const {
    Summary summary = this->GetSummary();
    // Has no verified data is set during initialization, so no need to update.
    if (summary.second != "Has no verified data") {
      stat.summaryf(summary.first, summary.second.c_str());
    }
    stat.add("Actual rate (Hz)", actual_hz_);
  }

 private:
  const double warn_hz_;
  const double error_hz_;
  double actual_hz_;

  std::deque<double> times_queue_;
  const uint32_t window_size_;
};


// Class to verify whether it is the expected frame_id
template <class MsgPtr>
class UnexpectedFrameId : public Interface<MsgPtr> {
 public:
  static typename Interface<MsgPtr>::SharedPtr Create(rclcpp::Node::SharedPtr node) {
    const std::string expected_frame_id =
        node->declare_parameter<std::string>("verifiers.unexpected_frame_id.expected_frame_id", "base_imu_frame");

    return typename Interface<MsgPtr>::SharedPtr(new UnexpectedFrameId<MsgPtr>(expected_frame_id));
  }
  explicit UnexpectedFrameId(const std::string& expected_frame_id) : expected_frame_id_(expected_frame_id) {}

  virtual void UpdateSummary(const MsgPtr& stamped_msg) {
    // If the value cannot be examined, it is judged not to be the job of this class, and no update is made
    if (stamped_msg == nullptr) {
      return;
    }

    if (stamped_msg->header.frame_id != expected_frame_id_) {
      this->SetAs(DiagnosticStatus::ERROR, "Unexpected frame ID");
    } else {
      this->SetAsOK();
    }
  }

  // No output of value type
  virtual void AddValues(diagnostic_updater::DiagnosticStatusWrapper& stat) const {
    Summary summary = this->GetSummary();
    // Has no verified data is set during initialization, so no need to update.
    if (summary.first != diagnostic_msgs::msg::DiagnosticStatus::OK && summary.second != "Has no verified data") {
      // Unexpected frame ID overlaps with other errors, so update with merge.
      stat.mergeSummaryf(summary.first, summary.second.c_str());
    }
  }

 private:
  const std::string expected_frame_id_;
};


// Create a validator that does not depend on the message type
template <class MsgPtr>
typename Interface<MsgPtr>::SharedPtr Create(std::string type, rclcpp::Node::SharedPtr node) {
  typename Interface<MsgPtr>::SharedPtr ptr;
  if (type == "disconnection") {
    ptr = Disconnection<MsgPtr>::Create(node);
  } else if (type == "unexpected_rate") {
    ptr = UnexpectedRate<MsgPtr>::Create(node);
  } else if (type == "unexpected_frame_id") {
    ptr = UnexpectedFrameId<MsgPtr>::Create(node);
  } else if (type.empty()) {
    throw std::invalid_argument("type parameter is empty");
  } else {
    throw std::invalid_argument(type + " is not implemented");
  }
  return ptr;
}

}  // namespace verifier
}  // namespace tmc_diag_updater_common
#endif  // TMC_DIAG_UPDATER_COMMON_TMC_DIAG_UPDATER_COMMON_VERIVIERS_HPP_
