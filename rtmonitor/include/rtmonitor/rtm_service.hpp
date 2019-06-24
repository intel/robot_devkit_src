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

#ifndef RTMONITOR__RTM_SERVICE_HPP_
#define RTMONITOR__RTM_SERVICE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rtmonitor_msgs/srv/req_loop_time.hpp"
#include "rtmonitor/rtm_publisher.hpp"

namespace rtmonitor
{

class RtmService : public rclcpp::Node
{
public:
  RtmService();
  ~RtmService();
  bool init();
  bool create_service_looptime();
  void handle_looptime(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<rtmonitor_msgs::srv::ReqLoopTime::Request> req,
    std::shared_ptr<rtmonitor_msgs::srv::ReqLoopTime::Response> res);

private:
  rclcpp::Service<rtmonitor_msgs::srv::ReqLoopTime>::SharedPtr loop_time_srv_;
  std::shared_ptr<RtmPublisher> rtm_pub_;
};

}  // namespace rtmonitor


#endif  // RTMONITOR__RTM_SERVICE_HPP_
