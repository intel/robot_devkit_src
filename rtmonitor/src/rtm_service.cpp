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
#include <map>

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

RtmPerfMetric* RtmService::init_perf_metric(std::string id)
{
  // Check if id already exists
  std::map<std::string, RtmPerfMetric *>::iterator it = perf_map_.find(id);
  if (it != perf_map_.end()) {
    RCLCPP_INFO(get_logger(), "Perf Metric Id:%s already registered", id.c_str());
    return nullptr;
  }

  // Create
  RtmPerfMetric * perf_metric = new RtmPerfMetric();
  perf_metric->id_ = id;
  perf_metric->iter_cnt_ = 0;
  perf_metric->start_ns_ = 0;
  perf_metric->stop_ns_ = 0;
  perf_metric->dur_ns_ = 0;

  perf_map_[id] = perf_metric;

  return perf_metric;
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

  return true;
}

void RtmService::handle_elapsed(
const std::shared_ptr<rmw_request_id_t> request_header,
const std::shared_ptr<rtmonitor_msgs::srv::ReqElapsed::Request> req,
std::shared_ptr<rtmonitor_msgs::srv::ReqElapsed::Response> res)
{
  RCLCPP_INFO(get_logger(), "handle_elapsed");
  uint64_t elapsed = 0;
  RtmPerfMetric * perf_metric;
  std::string id = req->req.id;

  std::map<std::string, RtmPerfMetric *>::iterator it = perf_map_.find(id);
  if (it != perf_map_.end()) {
    perf_metric = it->second;
  } else {
    printf("Error: No such Id monitored %s\n", id.c_str());
    perf_metric = init_perf_metric(id);
  }

  if (req->req.is_start) {
    perf_metric->start_ns_ = req->req.time_ns;
  } else {
    // TODO(lbegani): Check if id exists
    perf_metric->stop_ns_ = req->req.time_ns;

    // TODO(lbegani): Check if elapsed_start < elapsed_stop.
    if(perf_metric->stop_ns_ > perf_metric->start_ns_) {
      perf_metric->dur_ns_ = perf_metric->stop_ns_ - perf_metric->start_ns_;
      perf_metric->iter_cnt_++;

      // Save the calculated elapsed duration to a file
      uint32_t nsecs = (perf_metric->dur_ns_) % 1000000000;
      uint32_t secs = ((perf_metric->dur_ns_) - nsecs) / 1000000000;
      printf("%s:Iteration: %d Duration: %d secs %d nsecs\n", id.c_str(),
        perf_metric->iter_cnt_, secs, nsecs);
    }

    perf_metric->start_ns_ = 0;
    perf_metric->stop_ns_ = 0;
    perf_metric->dur_ns_ = 0;
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
