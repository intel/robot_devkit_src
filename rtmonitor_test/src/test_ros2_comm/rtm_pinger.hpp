// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TEST_ROS2_COMM__RTM_PINGER_HPP_
#define TEST_ROS2_COMM__RTM_PINGER_HPP_

#include <inttypes.h>

#include <string>
#include <memory>
#include <chrono>
#include <cmath>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "rtmonitor/rtmonitor.hpp"
#include "rtmonitor_msgs/msg/rtm_vector.hpp"
#include "qos_profile.hpp"


using namespace std::chrono_literals;

class RtmPinger : public rclcpp::Node
{
public:
  RtmPinger()
  : Node("rtmpinger")
  {
    RCLCPP_INFO(this->get_logger(), "RtmPinger");
  }

  RtmPinger(
    std::vector<std::string> topic_index_list, uint32_t test_dur, uint32_t qos_cfg,
    uint32_t rate, uint32_t len)
  : Node("rtmpinger")
  {
    RCLCPP_INFO(this->get_logger(), "RtmPinger");
    test_dur_ = test_dur;
    qos_cfg_ = qos_cfg;
    pub_rate_ = rate;
    msg_len_ = len;

    // TODO(lbegani): Parse topic indexes from input list
    (void)(topic_index_list);
    create_pub_sub("1");
  }

  ~RtmPinger()
  {
  }

  bool create_pub_sub(std::string topic_index)
  {
    rtm_.init("msg_RTT");

    // TODO(lbegani): make qos configurable
    // rclcpp::QoS qos(rclcpp::KeepLast(3));
    auto qos = getQosConfig(qos_cfg_);

    std::string ping_topic = "rtm_ping" + topic_index;
    std::string pong_topic = "rtm_pong" + topic_index;

    pub_ = this->create_publisher<rtmonitor_msgs::msg::RtmVector>(ping_topic, qos);

    sub_ = create_subscription<rtmonitor_msgs::msg::RtmVector>(pong_topic, 10,
        std::bind(&RtmPinger::pong_message, this, std::placeholders::_1));

    start(pub_rate_);

    return true;
  }

  void start(uint32_t rate)
  {
    uint32_t period_ms = 1000 / rate;
    timer_ =
      this->create_wall_timer(std::chrono::milliseconds(period_ms),
        std::bind(&RtmPinger::ping_message, this));

    // Run the test for specified amount of time
    test_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(test_dur_ * 60 * 1000),
        [this]() {
          RCLCPP_INFO(this->get_logger(), "Stop the test, output the results");
          this->stop();
          this->test_timer_->cancel();
        });
  }
  void stop()
  {
    // stop the timer
    this->timer_->cancel();
  }
  void ping_message()
  {
    RCLCPP_INFO(this->get_logger(), "-->Ping");
    rtmonitor_msgs::msg::RtmVector::UniquePtr msg(new rtmonitor_msgs::msg::RtmVector());
    msg->header.stamp = this->now();
    msg->index = index_++;
    msg->data.resize(msg_len_, 7);
    // int64_t src_time = RCUTILS_S_TO_NS(msg->header.stamp.sec) + (int64_t)msg->header.stamp.nanosec;
    // printf("%" PRId64 " :src\n", src_time);

    pub_->publish(std::move(msg));
  }
  void pong_message(const rtmonitor_msgs::msg::RtmVector::UniquePtr msg)
  {
    rtm_.calc_latency("msg_RTT", msg->header.stamp, this->now());
    RCLCPP_INFO(this->get_logger(), "<--Pong %" PRId64 " ", msg->index);

    // TODO(lbegani): Keep a list of missing indexes

    #if 0
    int64_t src_time = RCUTILS_S_TO_NS(msg->header.stamp.sec) + (int64_t)msg->header.stamp.nanosec;
    printf("%" PRId64 " :stamp\n", src_time);
    auto now = this->now();

    // int64_t dst_time = RCUTILS_S_TO_NS(now.seconds()) + (int64_t)now.nanoseconds();
    int64_t dst_time = (int64_t)now.nanoseconds();
    printf("%" PRId64 " :dst\n", dst_time);

    int64_t rtt = dst_time - src_time;
    printf("%" PRId64 " :rtt \n", rtt);
    #endif
  }

private:
  rclcpp::Publisher<rtmonitor_msgs::msg::RtmVector>::SharedPtr pub_;
  rclcpp::Subscription<rtmonitor_msgs::msg::RtmVector>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr test_timer_;
  uint64_t index_ = 1;
  uint32_t test_dur_;
  uint32_t qos_cfg_;
  uint32_t pub_rate_;
  uint32_t msg_len_;
  std::vector<std::string> topic_list_;
  rtmonitor::RealTimeMonitor rtm_;
};

#endif  // TEST_ROS2_COMM__RTM_PINGER_HPP_
