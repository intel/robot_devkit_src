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

#include <memory>
#include <string>

#include "rtmonitor/rtm_service.hpp"

namespace rtmonitor
{

RtmService::RtmService()
: Node("rtm_service")
{
  RCLCPP_INFO(get_logger(), "RtmService");
}

RtmService::~RtmService()
{
}

bool RtmService::init()
{
  create_service_looptime();
  create_service_elapsed();
  rtm_pub_ = std::make_shared<RtmPublisher>(shared_from_this());
  return true;
}

bool RtmService::create_service_looptime()
{
  auto handle_loop_time_callback = [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<rtmonitor_msgs::srv::ReqLoopTime::Request> request,
      std::shared_ptr<rtmonitor_msgs::srv::ReqLoopTime::Response> response) -> void
    {
      handle_looptime(request_header, request, response);
    };

  loop_time_srv_ = create_service<rtmonitor_msgs::srv::ReqLoopTime>("loop_time",
      handle_loop_time_callback);
  return true;
}

void RtmService::handle_looptime(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<rtmonitor_msgs::srv::ReqLoopTime::Request> req,
  std::shared_ptr<rtmonitor_msgs::srv::ReqLoopTime::Response> res)
{
  RCLCPP_INFO(get_logger(), "handle_looptime");
  rtmonitor_msgs::msg::LoopTime msg;
  msg.header.stamp = this->now();
  msg.topic = req->req.topic;
  msg.pub = req->req.pub;
  msg.rate = req->req.rate;
  msg.jitter = req->req.jitter;
  msg.iteration = req->req.iteration;
  msg.looptime = req->req.looptime;

  // publish looptime message
  rtm_pub_->publish_looptime(msg);
}

bool RtmService::create_service_elapsed()
{
  auto handle_elapsed_callback = [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<rtmonitor_msgs::srv::ReqElapsed::Request> request,
      std::shared_ptr<rtmonitor_msgs::srv::ReqElapsed::Response> response) -> void
    {
      handle_elapsed(request_header, request, response);
    };

  elapsed_srv_ = create_service<rtmonitor_msgs::srv::ReqElapsed>("elapsed",
      handle_elapsed_callback);

  elapsed_iter = 0;
  elapsed_start_ns = 0;
  elapsed_stop_ns = 0;

  return true;
}

void RtmService::handle_elapsed(
const std::shared_ptr<rmw_request_id_t> request_header,
const std::shared_ptr<rtmonitor_msgs::srv::ReqElapsed::Request> req,
std::shared_ptr<rtmonitor_msgs::srv::ReqElapsed::Response> res)
{
  RCLCPP_INFO(get_logger(), "handle_elapsed");
  uint64_t elapsed = 0;
  if (req->req.is_start) {
    // TODO(lbegani): Save the id if not already saved
    elapsed_id = req->req.id;
    elapsed_start_ns = req->req.time_ns;
  } else {
    // TODO(lbegani): Check if id exists
    elapsed_stop_ns = req->req.time_ns;

    // TODO(lbegani): Check if elapsed_start < elapsed_stop.
    elapsed = elapsed_stop_ns - elapsed_start_ns;
    elapsed_iter++;

    // Save the calculated elapsed duration to a file
    uint32_t nsecs = (elapsed) % 1000000000;
    uint32_t secs = ((elapsed) - nsecs) / 1000000000;
    printf("Iteration: %d Duration: %d secs %d nsecs\n", elapsed_iter, secs, nsecs);
  }
}

}  // namespace rtmonitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto service = std::make_shared<rtmonitor::RtmService>();
  service->init();
  rclcpp::spin(service);
  rclcpp::shutdown();
  return 0;
}
