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

#ifndef TEST_ROS2_COMM__RTM_PONGER_HPP_
#define TEST_ROS2_COMM__RTM_PONGER_HPP_

#include <string>
#include <memory>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"

class RtmPonger : public rclcpp::Node
{
public:
  RtmPonger()
  : Node("rtmponger")
  {
    create_sub_pub("1");
  }

  explicit RtmPonger(std::vector<std::string> topic_index_list)
  : Node("rtmponger")
  {
    // TODO(lbegani): Parse topic indexes from input list
    (void)(topic_index_list);
    create_sub_pub("1");
  }

  ~RtmPonger()
  {
  }

  bool create_sub_pub(std::string topic_index)
  {
    std::string ping_topic = "rtm_ping" + topic_index;
    std::string pong_topic = "rtm_pong" + topic_index;

    // create subscriber
    sub_ = create_subscription<rtmonitor_msgs::msg::RtmVector>(ping_topic, 10,
        std::bind(&RtmPonger::ping_message, this, std::placeholders::_1));

    // create publisher
    // TODO(lbegani): make qos configurable
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<rtmonitor_msgs::msg::RtmVector>(pong_topic, qos);

    return true;
  }

  // Incoming message
  void ping_message(const rtmonitor_msgs::msg::RtmVector::UniquePtr msg)
  {
    // printf("%d %d %d\n", msg->data[0], msg->data[234], msg->data[5345]);
    rtmonitor_msgs::msg::RtmVector::UniquePtr msg2(new rtmonitor_msgs::msg::RtmVector());
    msg2->header.stamp = msg->header.stamp;
    msg2->index = msg->index;
    pub_->publish(std::move(msg2));
  }

private:
  rclcpp::Publisher<rtmonitor_msgs::msg::RtmVector>::SharedPtr pub_;
  rclcpp::Subscription<rtmonitor_msgs::msg::RtmVector>::SharedPtr sub_;
};

#endif  // TEST_ROS2_COMM__RTM_PONGER_HPP_
