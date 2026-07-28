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
/**
 * @file subscriber.hpp
 * @brief Provides a cached Subscriber, etc.
 * @auther Fukukazu Kawata
 *
 *
 */
#ifndef TEST_UTILS_SUBSCRIBER_HPP_
#define TEST_UTILS_SUBSCRIBER_HPP_
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "util_function.hpp"

namespace test_utils {

/**
 * @brief Cached Subscriber
 *
 * Start/stop subscription and clear buffer
 *
 * @tparam M Message type
 */
template <class M>
class CacheSubscriber {
 public:
  /**
   * @brief Alias for reference-counted smart pointer
   */
  using SharedPtr = std::shared_ptr<CacheSubscriber>;

  /**
   * @brief Constructor
   *
   * @param nh Node handle used to create the subscriber
   * @param topic_name Name of the topic to subscribe to
   * @param queue_size Queue size for the Subscriber
   */
  CacheSubscriber(rclcpp::Node::SharedPtr node, const std::string& topic_name, const uint32_t queue_size)
      : topic_name_(topic_name), queue_size_(queue_size), node_(node) {
    if (topic_name_.empty()) {
      throw std::invalid_argument("Topic name is empty");
    }
    if (queue_size_ == 0) {
      throw std::invalid_argument("Queue size must be larger than 0");
    }
  }

  /**
   * @brief Destructor (default)
   */
  virtual ~CacheSubscriber() = default;

  /**
   * @brief Start subscription
   */
  void StartCaching() {
    sub_ = node_->create_subscription<M>(topic_name_, queue_size_,
                                         std::bind(&CacheSubscriber<M>::CacheMessage, this, std::placeholders::_1));
  }

  /**
   * @brief Stop subscription
   */
  void StopCaching() {
    sub_.reset();
    sub_ = nullptr;
  }

  /**
   * @brief Initialize cache
   */
  void ClearCache() { std::vector<M>().swap(cache_); }

  /**
   * @brief Check if the subscribed topic is being published
   *
   * @return Whether the topic is being published or not
   */
  bool IsPublished() const { return sub_->get_publisher_count() != 0; }

  /**
   * @brief Check if the subscribed topic is being published (wait for a maximum specified time)
   *
   * @param wait_sec Waiting time (sec)
   *
   * @return Whether the topic is being published or not
   */
  bool IsPublished(const double wait_sec) const {
    return WaitUntil([&]() { return sub_->get_publisher_count() != 0; }, wait_sec);
  }

  /**
   * @brief Get the length of the cache
   *
   * @return Length of the cache
   */
  uint32_t GetCacheLength() const { return static_cast<uint32_t>(cache_.size()); }

  /**
   * @brief Get a copy of the cache
   *
   * @return Copy of the cache
   */
  std::vector<M> cache() const { return cache_; }

  /**
   * @brief Get the latest message in the cache
   *
   * @return Latest message
   */
  M GetLatestMessage() const { return cache_.back(); }

 private:
  /**
   * @brief Add a message to the cache. Bound as the Subscriber's callback
   *
   * @param msg Message
   */
  void CacheMessage(const M& msg) { cache_.push_back(msg); }

  const std::string topic_name_;
  const uint32_t queue_size_;
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Subscription<M>::SharedPtr sub_;
  std::vector<M> cache_;
};

}  // end of namespace test_utils
#endif  // TEST_UTILS_SUBSCRIBER_HPP_
