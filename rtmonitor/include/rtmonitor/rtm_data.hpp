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

#ifndef RTMONITOR__RTM_DATA_HPP_
#define RTMONITOR__RTM_DATA_HPP_

#include <iostream>
#include <chrono>
#include <functional>
#include <string>

namespace rtmonitor
{
class RtmData
{
public:
  RtmData()
  : init_(false), max_perf_time_(0, 0), min_perf_time_(0, 0),
    iter_cnt_(0), start_perf_time_(0, 0), stop_perf_time_(0, 0),
    perf_time_(0, 0) {}
  ~RtmData() {}

  bool init_;
  std::string event_id_;
  std::function<void(int, rclcpp::Duration)> overrun_cb_;
  FILE * log_file_;

  rclcpp::Duration max_perf_time_;
  rclcpp::Duration min_perf_time_;

  uint32_t iter_cnt_;
  rclcpp::Time start_perf_time_;
  rclcpp::Time stop_perf_time_;
  rclcpp::Duration perf_time_;
};

}  // namespace rtmonitor

#endif  // RTMONITOR__RTM_DATA_HPP_
