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

#include <sys/resource.h>

#include <string>
#include <map>
#include <memory>

#include "rtmonitor/rtmonitor.hpp"

namespace rtmonitor
{

void RealTimeMonitor::print_duration(FILE * log_file_, uint32_t iter, rclcpp::Duration dur) const
{
  uint32_t nsecs = (dur.nanoseconds()) % 1000000000;
  uint32_t secs = ((dur.nanoseconds()) - nsecs) / 1000000000;
  fprintf(log_file_, "Iteration: %d Duration: %d secs %d nsecs\n", iter, secs, nsecs);
}

void RealTimeMonitor::print_duration(FILE * log_file_, uint32_t iter, uint64_t dur) const
{
  uint32_t nsecs = dur % 1000000000;
  uint32_t secs = (dur - nsecs) / 1000000000;
  fprintf(log_file_, "Iteration: %d Duration: %d secs %d nsecs\n", iter, secs, nsecs);
}

void RealTimeMonitor::print_metrics(FILE * log_file_) const
{
  struct rusage r_usage;

  if (getrusage(RUSAGE_SELF, &r_usage) != 0) {
    return;
  }

  fprintf(log_file_, "Minor pagefaults: %lu\n", r_usage.ru_minflt);
  fprintf(log_file_, "Major pagefaults: %lu\n", r_usage.ru_majflt);
  fprintf(log_file_, "Memory usage: %lu\n", r_usage.ru_maxrss);
}

RealTimeMonitor::RealTimeMonitor()
{
}

RealTimeMonitor::~RealTimeMonitor()
{
  for (std::map<std::string, RtmData *>::iterator it = rtd_map_.begin(); it != rtd_map_.end();
    ++it)
  {
    if (it->second->log_file_) {
      fclose(it->second->log_file_);
    }

    delete it->second;
  }
}

bool RealTimeMonitor::init(rclcpp::Node::SharedPtr node, bool pub_metric)
{
  // TODO(lbegani): Check if client already exists
  rtm_client_ = std::make_shared<RtmClient>(node);
  // TODO(lbegani): Check if client created successfully
  pub_metric_ = pub_metric;
  return true;
}

bool RealTimeMonitor::init(rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node, bool pub_metric)
{
  // TODO(lbegani): Check if client already exists
  rtm_client_ = std::make_shared<RtmClient>(lc_node);
  // TODO(lbegani): Check if client created successfully
  pub_metric_ = pub_metric;
  return true;
}

bool RealTimeMonitor::deinit()
{
  rtm_client_.reset();
  return true;
}

bool RealTimeMonitor::register_callback(
  std::string id, rclcpp::Duration exp_perf_ns, rclcpp::Duration exp_jitter_ns,
  std::function<void(uint32_t iter_num, rclcpp::Duration perf_ns)> cb)
{
  // If id is not created, create id
  RtmData * rtd = get_metrics_data(id);
  if(!rtd) {
    add_metrics(id);
    rtd = get_metrics_data(id);
  }

  // Create cb struct and put it to id data
  rtd->cb_ = new RtmCallback();
  rtd->cb_->id_ = id;
  rtd->cb_->overrun_cb_ = cb;
  rtd->cb_->perf_ns_ = exp_perf_ns.nanoseconds();
  rtd->cb_->jitter_ns_ = exp_jitter_ns.nanoseconds();

  return true;
}

bool RealTimeMonitor::deregister_callback(std::string id)
{
  // If id is created and callback is registered, delete registeration
  RtmData * rtd = get_metrics_data(id);
  if(!rtd)
    return false;

  if(rtd->cb_) {
    delete rtd->cb_;
    rtd->cb_ = nullptr;
  }

  return true;
}

bool RealTimeMonitor::add_metrics(std::string id)
{
  if(get_metrics_data(id) != nullptr) {
    // print log
    return false;
  }

  RtmData * rtd = new RtmData();
  rtd->cb_ = nullptr;
  rtd->event_id_ = id;

  // TODO(lbegani): Check if file exists
  std::string filename = "/tmp/log_" + id + ".txt";
  rtd->log_file_ = fopen(filename.c_str(), "w");
  if (rtd->log_file_ == NULL) {
    printf("Error: Could not open log file");
  }

  rtd->perf_ = new RtmPerfMetric();
  rtd->perf_->id_ = id;
  rtd->perf_->iter_cnt_ = 0;
  rtd->perf_->start_ns_ = 0;
  rtd->perf_->stop_ns_ = 0;
  rtd->perf_->dur_ns_ = 0;

  rtd_map_[id] = rtd;

  return true;
}

bool RealTimeMonitor::remove_metrics(std::string id)
{
  RtmData * rtd = get_metrics_data(id);
  if(rtd == nullptr)
    return false;

  fclose(rtd->log_file_);

  // Remove entry from map

  // Delete the structure
  if(rtd->cb_)
    delete rtd->cb_;

  if(rtd->perf_)
    delete rtd->perf_;

  delete rtd;

  return true;
}

RtmData* RealTimeMonitor::get_metrics_data(std::string id)
{
  RtmData * rtd = nullptr;
  std::map<std::string, RtmData *>::iterator it = rtd_map_.find(id);
  if (it != rtd_map_.end()) {
    rtd = it->second;
  } else {
    printf("No such metric Id registered: %s\n", id.c_str());
    rtd = nullptr;
  }

  return rtd;
}

rclcpp::Duration RealTimeMonitor::calc_looptime(std::string id, rclcpp::Time now)
{
  // If id is not created, create id
  RtmData * rtd = get_metrics_data(id);
  if(!rtd) {
    add_metrics(id);
    rtd = get_metrics_data(id);
  }

  uint64_t looptime = 0;
  uint64_t jitter = 0;

  if(rtd->perf_->start_ns_ == 0)
    rtd->perf_->start_ns_ = now.nanoseconds();

  rtd->perf_->stop_ns_ = now.nanoseconds();

  looptime = rtd->perf_->stop_ns_ - rtd->perf_->start_ns_;

  // fprintf(rtd->log_file_, "Iteration: %d ", rtd->iter_cnt_);
  print_duration(rtd->log_file_, rtd->perf_->iter_cnt_, looptime);

  if (rtd->cb_) {
    // calculate difference between looptime and expected time
    jitter = (looptime > rtd->cb_->perf_ns_) ?
                looptime - rtd->cb_->perf_ns_ : rtd->cb_->perf_ns_ - looptime;
    // call the callback if diff more than jitter
    if(jitter > rtd->cb_->jitter_ns_ && rtd->perf_->iter_cnt_) {
      printf("Deadline Missed- looptime:%ld Expected time:%ld \n", looptime, rtd->cb_->perf_ns_);
      printf("Jitter:%ld Expected Jitter:%ld\n", jitter, rtd->cb_->jitter_ns_);
      rtd->cb_->overrun_cb_(rtd->perf_->iter_cnt_, rclcpp::Duration(looptime));
    }
  }

  rtd->perf_->dur_ns_ = looptime;

  // Call the client API to publish data
  if (pub_metric_ && rtm_client_) {
    rtm_client_->request_perfmetric(rtd->perf_);
  }

  rtd->perf_->start_ns_ = now.nanoseconds();
  rtd->perf_->iter_cnt_++;

  return rclcpp::Duration(looptime);
}

rclcpp::Duration RealTimeMonitor::calc_latency(
  std::string id,
  const builtin_interfaces::msg::Time & ctime,
  rclcpp::Time now)
{
  // If id is not created, create id
  RtmData * rtd = get_metrics_data(id);
  if(!rtd) {
    add_metrics(id);
    rtd = get_metrics_data(id);
  }

  rclcpp::Duration latency(0, 0);

  rclcpp::Time msg_time(ctime.sec, ctime.nanosec, now.get_clock_type());

  latency = now - msg_time;
  print_duration(rtd->log_file_, rtd->perf_->iter_cnt_, latency);
  rtd->perf_->iter_cnt_++;
  return latency;
}

rclcpp::Duration RealTimeMonitor::calc_elapsed(std::string id, bool is_start, rclcpp::Time now)
{
  // If id is not created, create id
  RtmData * rtd = get_metrics_data(id);
  if(!rtd) {
    add_metrics(id);
    rtd = get_metrics_data(id);
  }

  uint64_t elapsed = 0;

  if (is_start) {
    rtd->perf_->start_ns_ = now.nanoseconds();
  } else {
    rtd->perf_->stop_ns_ = now.nanoseconds();

    if(rtd->perf_->stop_ns_ >= rtd->perf_->start_ns_
        && rtd->perf_->start_ns_ > 0) {
      elapsed = rtd->perf_->stop_ns_ - rtd->perf_->start_ns_;
      print_duration(rtd->log_file_, rtd->perf_->iter_cnt_, elapsed);
      rtd->perf_->iter_cnt_++;
    } else {
      // RCLCPP_INFO(get_logger(), "Error: %s: Start and Stop out of sync", id.c_str());
    }

    rtd->perf_->start_ns_ = 0;
    rtd->perf_->stop_ns_ = 0;
    rtd->perf_->dur_ns_ = 0;
  }

  return rclcpp::Duration(elapsed);
}

bool RealTimeMonitor::calc_elapsed_g(std::string id, bool is_start, rclcpp::Time now)
{
  bool ret = false;

  // Call the client API
  if (rtm_client_) {
    ret = rtm_client_->request_elapsed(id, is_start, now);
  }

  return ret;
}

}  // namespace rtmonitor
