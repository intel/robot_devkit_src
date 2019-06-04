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

#pragma once

#include <iostream>
#include <chrono>
#include <functional>

namespace rtmonitor
{
class RtmData
{
public:
  RtmData()
  : iter_cnt_(0), prev_looptime_(0, 0), acceptable_looptime_(0, 0) {}
  ~RtmData() {}
  bool init_;
  uint32_t iter_cnt_;
  uint32_t rate_;
  uint32_t jitter_margin_;
  std::function<void(int, rclcpp::Duration)> overrun_cb_;

  rclcpp::Time prev_looptime_;
  rclcpp::Duration acceptable_looptime_;

  rclcpp::Time elapsed_start;
  rclcpp::Time elapsed_stop;

  FILE * log_file_;
};

}  // namespace rtmonitor

#endif  // RTMONITOR__RTM_DATA_HPP_
