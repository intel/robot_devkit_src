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

struct RtmCallback {
  std::string id_;
  std::function<void(uint32_t, rclcpp::Duration)> overrun_cb_;
  uint64_t perf_ns_;
  uint64_t jitter_ns_;
};

struct RtmPerfMetric {
  std::string id_;
  uint32_t iter_cnt_;
  uint64_t start_ns_;
  uint64_t stop_ns_;
  uint64_t dur_ns_;
};

class RtmData
{
public:
  RtmData() {}
  ~RtmData() {}

  std::string event_id_;
  FILE * log_file_;
  RtmCallback * cb_;
  RtmPerfMetric * perf_;
};

}  // namespace rtmonitor

#endif  // RTMONITOR__RTM_DATA_HPP_
