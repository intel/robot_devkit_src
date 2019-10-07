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

#ifndef RTMONITOR__RTM_CLIENT_HPP_
#define RTMONITOR__RTM_CLIENT_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rtmonitor/rtm_data.hpp"
#include "rtmonitor_msgs/srv/req_loop_time.hpp"
#include "rtmonitor_msgs/srv/req_elapsed.hpp"

namespace rtmonitor
{

class RtmClient
{
public:
  explicit RtmClient(rclcpp::Node::SharedPtr node);
  explicit RtmClient(rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node);
  ~RtmClient();
  bool create_client_looptime(rclcpp::Node::SharedPtr node);
  bool create_client_elapsed(rclcpp::Node::SharedPtr node);
  bool lc_create_client_elapsed(rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node);
  bool request_looptime(RtmData * rtd);
  bool request_elapsed(std::string id, bool is_start, rclcpp::Time now);

private:
  rclcpp::Client<rtmonitor_msgs::srv::ReqLoopTime>::SharedPtr loop_time_client_;
  rclcpp::Client<rtmonitor_msgs::srv::ReqElapsed>::SharedPtr elapsed_client_;
};

}  // namespace rtmonitor


#endif  // RTMONITOR__RTM_CLIENT_HPP_
