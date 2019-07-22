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

#ifndef TEST_ROS2_COMM__QOS_PROFILE_HPP_
#define TEST_ROS2_COMM__QOS_PROFILE_HPP_

#include "rclcpp/rclcpp.hpp"

#define QOS_CONFIG_DEFAULT 0
#define QOS_CONFIG_LARGE_DATA 1
#define QOS_CONFIG_BEST_EFFORT 2
#define QOS_CONFIG_RELIABLE 3

rclcpp::QoS
getQosConfig(const uint32_t qos_cfg)
{
  printf("header function\n");
  rclcpp::QoS qos(rclcpp::KeepLast(3));
  switch (qos_cfg) {
    case QOS_CONFIG_DEFAULT:
      qos = qos.keep_last(3);
      qos = qos.reliable();
      break;
    case QOS_CONFIG_LARGE_DATA:
      qos = qos.keep_last(3);
      break;
    case QOS_CONFIG_BEST_EFFORT:
      qos = qos.keep_last(1);
      qos = qos.best_effort();
      break;
    case QOS_CONFIG_RELIABLE:
      qos = qos.keep_all();
      qos = qos.reliable();
      break;
  }

  return qos;
}

#endif  // TEST_ROS2_COMM__QOS_PROFILE_HPP_
