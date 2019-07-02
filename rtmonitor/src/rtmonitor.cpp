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

  // TODO(lbegani): Delete the RtmData structs.
}

bool RealTimeMonitor::init(std::string id)
{
  // TODO(lbegani): Check if the id already exists.

  RtmData * rtd = new RtmData();

  rtd->event_id_ = id;

  // TODO(lbegani): Check if file exists
  std::string filename = "/tmp/log_" + id + ".txt";
  rtd->log_file_ = fopen(filename.c_str(), "w");
  if (rtd->log_file_ == NULL) {
    printf("Error: Could not open log file");
  }

  rtd->init_ = false;

  rtd_map_[id] = rtd;

  return true;
}

bool RealTimeMonitor::init(rclcpp::Node::SharedPtr node, std::string id)
{
  if (!init(id)) {
    return false;
  }

  rtm_client_ = std::make_shared<RtmClient>(node);
  // Check if client created successfully
  return true;
}

bool RealTimeMonitor::init(
  std::string id, uint32_t rate, uint32_t jitter_margin,
  std::function<void(int iter_num, rclcpp::Duration looptime)> cb)
{
  if (!init(id)) {
    return false;
  }

  RtmData * rtd;
  std::map<std::string, RtmData *>::iterator it = rtd_map_.find(id);
  if (it != rtd_map_.end()) {
    rtd = it->second;
  } else {
    printf("Error: Initialization error %s", id.c_str());
    return false;
  }

  // rtd->rate_ = rate;
  // rtd->jitter_margin_ = jitter_margin;

  rtd->overrun_cb_ = cb;

  uint32_t looptime_ns = 1000000000 / rate;
  uint32_t jitter_ns = (looptime_ns * jitter_margin) / 100;
  uint32_t max_looptime_ns = looptime_ns + jitter_ns;
  uint32_t min_looptime_ns = looptime_ns - jitter_ns;
  // rtd->acceptable_looptime_ = rclcpp::Duration(0, desired_looptime_ns);
  rtd->max_perf_time_ = rclcpp::Duration(0, max_looptime_ns);
  rtd->min_perf_time_ = rclcpp::Duration(0, min_looptime_ns);

  fprintf(rtd->log_file_, "Desired looptime:%ld ns \n",
    long(rtd->max_perf_time_.nanoseconds()));

  return true;
}

bool RealTimeMonitor::init(
  rclcpp::Node::SharedPtr node, std::string id, uint32_t rate, uint32_t jitter_margin,
  std::function<void(int iter_num, rclcpp::Duration looptime)> cb)
{
  if (!init(id, rate, jitter_margin, cb)) {
    return false;
  }

  rtm_client_ = std::make_shared<RtmClient>(node);
  // Check if client created successfully
  return true;
}

bool RealTimeMonitor::deinit(std::string id)
{
  // TODO(lbegani): Remove the id/data from the map
  (void)(id);
  return true;
}

rclcpp::Duration RealTimeMonitor::calc_looptime(std::string id, rclcpp::Time now)
{
  RtmData * rtd;
  rclcpp::Duration looptime(0, 0);
  // TODO(lbegani): Put id->rtd in a common util function
  std::map<std::string, RtmData *>::iterator it = rtd_map_.find(id);
  if (it != rtd_map_.end()) {
    rtd = it->second;
  } else {
    printf("Error: No such Id monitored %s\n", id.c_str());
    return looptime;
  }

/*
  if(now < prev_looptime_) {
    printf("Invalid argument");
    return -1;
  }
*/

  rtd->stop_perf_time_ = now;

  if (rtd->init_) {
    looptime = rtd->stop_perf_time_ - rtd->start_perf_time_;
  } else {
    rtd->init_ = true;
  }

  // fprintf(rtd->log_file_, "Iteration: %d ", rtd->iter_cnt_);
  print_duration(rtd->log_file_, rtd->iter_cnt_, looptime);

  if (rtd->overrun_cb_) {
    if (looptime > rtd->max_perf_time_) {
      rtd->overrun_cb_(rtd->iter_cnt_, looptime);
      print_metrics(rtd->log_file_);
    }
  }

  rtd->perf_time_ = looptime;

  // Call the client API
  if (rtm_client_) {
    rtm_client_->request_looptime(rtd);
  }

  rtd->start_perf_time_ = now;
  rtd->iter_cnt_++;

  return looptime;
}

rclcpp::Duration RealTimeMonitor::calc_latency(
  std::string id,
  const builtin_interfaces::msg::Time & time,
  rclcpp::Time now)
{
  rclcpp::Duration latency(0, 0);
  RtmData * rtd;
  std::map<std::string, RtmData *>::iterator it = rtd_map_.find(id);
  if (it != rtd_map_.end()) {
    rtd = it->second;
  } else {
    printf("Error: No such Id monitored %s\n", id.c_str());
    return latency;
  }

  rclcpp::Time msg_time(time.sec, time.nanosec, now.get_clock_type());

  latency = now - msg_time;
  print_duration(rtd->log_file_, rtd->iter_cnt_, latency);
  rtd->iter_cnt_++;
  return latency;
}

rclcpp::Duration RealTimeMonitor::calc_elapsed(std::string id, bool is_start, rclcpp::Time now)
{
  rclcpp::Duration elapsed(0, 0);
  RtmData * rtd;
  std::map<std::string, RtmData *>::iterator it = rtd_map_.find(id);
  if (it != rtd_map_.end()) {
    rtd = it->second;
  } else {
    printf("Error: No such Id monitored %s\n", id.c_str());
    return elapsed;
  }

  if (is_start) {
    rtd->start_perf_time_ = now;
  } else {
    rtd->stop_perf_time_ = now;

    // TODO(lbegani): Check if elapsed_start < elapsed_stop.
    elapsed = rtd->stop_perf_time_ - rtd->start_perf_time_;

    print_duration(rtd->log_file_, rtd->iter_cnt_, elapsed);
    rtd->iter_cnt_++;
    // TODO(lbegani): reset elapsed_start and elapsed_stop.
  }

  return elapsed;
}

}  // namespace rtmonitor
