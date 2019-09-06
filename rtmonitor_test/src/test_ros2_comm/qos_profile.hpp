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

#include <chrono>

#include "rclcpp/rclcpp.hpp"

#define QOS_DEFAULT             0
#define QOS_KEEP_ALL            1
#define QOS_KEEP_LAST           2
#define QOS_RELIABLE            3
#define QOS_BEST_EFFORT         4
#define QOS_DURABILITY_VOLATILE 5
#define QOS_TRANSIENT_LOCAL     6
#define QOS_LIVELINESS          7
#define QOS_LIFESPAN            8
#define QOS_DEADLINE            9
#define QOS_LARGE_DATA          10

using namespace std::chrono_literals;

rclcpp::QoS
getQosConfig(const uint32_t qos_cfg)
{
  printf("QoS Config:%d \n", qos_cfg);
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  std::chrono::milliseconds duration = 1s;
  switch (qos_cfg) {
    case QOS_KEEP_ALL:
      qos.keep_all();
      break;
    case QOS_KEEP_LAST:
      qos.keep_last(10);
      break;
    case QOS_RELIABLE:
      qos.reliable();
      break;
    case QOS_BEST_EFFORT:
      qos.best_effort();
      break;
    case QOS_DURABILITY_VOLATILE:
      qos.durability_volatile();
      break;
    case QOS_TRANSIENT_LOCAL:
      qos.transient_local();
      break;
    case QOS_LIVELINESS:
      // const std::chrono::milliseconds lease_duration = 1s;
      duration = 1s;
      qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
      qos.liveliness_lease_duration(duration);
      break;
    case QOS_LIFESPAN:
      // std::chrono::milliseconds lifespan_duration = 10ms;
      duration = 10ms;
      qos
      .reliable()
      .transient_local()
      .lifespan(duration);
      break;
    case QOS_DEADLINE:
      // std::chrono::milliseconds deadline_duration = 10ms;
      duration = 10ms;
      qos.deadline(duration);
      break;
    case QOS_DEFAULT:
      qos = qos.keep_last(3);
      qos = qos.reliable();
      break;
    case QOS_LARGE_DATA:
      qos = qos.keep_last(3);
      break;
    default:
      qos = qos.keep_last(3);
      qos = qos.reliable();
  }

  return qos;
}

#endif  // TEST_ROS2_COMM__QOS_PROFILE_HPP_
