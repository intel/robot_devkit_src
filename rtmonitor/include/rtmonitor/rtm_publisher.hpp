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

#ifndef RTMONITOR__RTM_PUBLISHER_HPP_
#define RTMONITOR__RTM_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rtmonitor_msgs/msg/loop_time.hpp"

namespace rtmonitor
{

class RtmPublisher
{
public:
  explicit RtmPublisher(rclcpp::Node::SharedPtr node);
  ~RtmPublisher();
  bool create_publisher_looptime(rclcpp::Node::SharedPtr node);
  bool publish_looptime(rtmonitor_msgs::msg::LoopTime msg);

private:
  rclcpp::Publisher<rtmonitor_msgs::msg::LoopTime>::SharedPtr pub_looptime_;
};

}  // namespace rtmonitor


#endif  // RTMONITOR__RTM_PUBLISHER_HPP_
