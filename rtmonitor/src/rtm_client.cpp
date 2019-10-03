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

#include "rtmonitor/rtm_client.hpp"

namespace rtmonitor
{

RtmClient::RtmClient(rclcpp::Node::SharedPtr node)
{
  create_client_looptime(node);
  create_client_elapsed(node);
}

RtmClient::~RtmClient()
{
}

bool RtmClient::create_client_looptime(rclcpp::Node::SharedPtr node)
{
  loop_time_client_ = node->create_client<rtmonitor_msgs::srv::ReqLoopTime>("loop_time");

  while (!loop_time_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
      return false;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
  }

  return true;
}

bool RtmClient::request_looptime(RtmData * rtd)
{
  auto request = std::make_shared<rtmonitor_msgs::srv::ReqLoopTime::Request>();
  // request->header.stamp = rclcpp::now();
  request->req.topic = rtd->event_id_;
  request->req.pub = true;
  request->req.rate = 0;
  request->req.jitter = 0;
  request->req.iteration = rtd->iter_cnt_;
  request->req.looptime = rtd->perf_time_.nanoseconds();

  auto result_future = loop_time_client_->async_send_request(request);

#if 0
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    return 1;
  }

  auto result = result_future.get();
  RCLCPP_INFO(node->get_logger(), "result of %" PRId64 " + %" PRId64 " = %" PRId64,
    request->a, request->b, result->sum);
#endif

  return true;
}

bool RtmClient::create_client_elapsed(rclcpp::Node::SharedPtr node)
{
  elapsed_client_ = node->create_client<rtmonitor_msgs::srv::ReqElapsed>("elapsed");

  while (!elapsed_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for elapsed service to appear.");
      return false;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for elapsed service to appear...");
  }

  return true;
}

bool RtmClient::request_elapsed(std::string id, bool is_start, rclcpp::Time now)
{
  // TODO(lbegani): check if elapsed service is running

  auto request = std::make_shared<rtmonitor_msgs::srv::ReqElapsed::Request>();

  request->req.id = id;
  request->req.is_start = is_start;
  request->req.time_ns = now.nanoseconds();

  auto result_future = elapsed_client_->async_send_request(request);

  // TODO(lbegani): Check if elapsed need to be fetched from service. Ignoring it now to save time.

  return true;
}


}  // namespace rtmonitor
