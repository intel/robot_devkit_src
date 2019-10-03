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

#ifndef RTMONITOR__RTMONITOR_HPP_
#define RTMONITOR__RTMONITOR_HPP_

#pragma once

#include <string>
#include <map>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rtmonitor/rtm_data.hpp"
#include "rtmonitor/rtm_client.hpp"
#include "rtmonitor_msgs/msg/loop_time.hpp"
#include "rtmonitor_msgs/msg/elapsed.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace rtmonitor
{

/**
 *  The RealTimeMonitor class is used to measure real-time performance metrics of a software.
 */

class RealTimeMonitor
{
public:
  RealTimeMonitor();
  ~RealTimeMonitor();

  /**
   *  Initialize.
   *
   *  @param[in] id Identifier.
   *
   *  @return Status of request.
   */
  bool init(std::string id);

  /**
   *  Initialize with upper and lower limits.
   *
   *  @param[in] id Identifier.
   *
   *  @return Status of request.
   */
  // bool init(std::string id, rclcpp::Duration max, rclcpp::Duration min);

  /**
   *  Initialize to publish RT metrics on ROS2 topic.
   *
   *  @param[in] node Node Pointer.
   *  @param[in] id Identifier.
   *
   *  @return Status of request.
   */
  bool init(rclcpp::Node::SharedPtr node, std::string id);

  /**
   *  Initialize for inter-process performance metrics for lifecycle node.
   *
   *  @param[in] lc_node LifecycleNode Pointer.
   *  @param[in] id Identifier.
   *
   *  @return Status of request.
   */
  bool init(rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node, std::string id);

  /**
   *  Initialize to publish RT metrics on ROS2 topic.
   *
   *  @param[in] node Node Pointer.
   *  @param[in] id Identifier.
   *
   *  @return Status of request.
   */
  // bool init(rclcpp::Node::SharedPtr node, std::string id, rclcpp::Duration max, rclcpp::Duration min);

  /**
   *  Initialize to receive callback on missed deadlines.
   *
   *  @param[in] id Identifier.
   *  @param[in] rate Rate set for loop.
   *  @param[in] jitter_margin Deviation acceptable in percentage.
   *  @param[in] cb Callback to be called on missed deadlines.
   *
   *  @return Status of request.
   */
  bool init(
    std::string id, uint32_t rate, uint32_t jitter_margin,
    std::function<void(int iter_num, rclcpp::Duration looptime)> cb);

  /**
   *  Initialize to receive callback on missed deadlines and publish RT metrics.
   *
   *  @param[in] node Node Pointer.
   *  @param[in] id Identifier.
   *  @param[in] rate Rate set for loop.
   *  @param[in] jitter_margin Deviation acceptable in percentage.
   *  @param[in] cb Callback to be called on missed deadlines.
   *
   *  @return Status of request.
   */
  bool init(
    rclcpp::Node::SharedPtr node, std::string id, uint32_t rate, uint32_t jitter_margin,
    std::function<void(int iter_num, rclcpp::Duration looptime)> cb);


  /**
   *  Deinitialize.
   *
   *  @param[in] id Identifier.
   *
   *  @return Status of request.
   */
  bool deinit(std::string id);

  /**
   *  Calculate Looptime.
   *
   *  @param[in] id Identifier.
   *  @param[in] now Current Time.
   *
   *  @return Looptime in rclcpp::Duration.
   */
  rclcpp::Duration calc_looptime(std::string id, rclcpp::Time now);

  /**
   *  Calculate Message Latency.
   *
   *  @param[in] id Identifier.
   *  @param[in] time Timestamp in message header.
   *  @param[in] now Current Time.
   *
   *  @return Latency in rclcpp::Duration.
   */
  rclcpp::Duration calc_latency(
    std::string id, const builtin_interfaces::msg::Time & time,
    rclcpp::Time now);

  /**
   *  Calculate time elapsed between two point.
   *
   *  @param[in] id Identifier.
   *  @param[in] is_start Flag to specify if start point or end point.
   *  @param[in] now Current Time.
   *
   *  @return Elapsed time in rclcpp::Duration.
   */
  rclcpp::Duration calc_elapsed(std::string id, bool is_start, rclcpp::Time now);

  /**
   *  Calculate time elapsed between two point in different processes.
   *
   *  @param[in] id Identifier.
   *  @param[in] is_start Flag to specify if start point or end point.
   *  @param[in] now Current Time.
   *
   *  @return Status of request.
   */
  bool calc_elapsed_g(std::string id, bool is_start, rclcpp::Time now);

private:
  void print_duration(FILE * log_file_, uint32_t iter, rclcpp::Duration dur) const;
  void print_metrics(FILE * log_file_) const;
  int create_publisher();
  int destroy_publisher();
  int publish_looptime(RtmData * rtd);

  std::map<std::string, RtmData *> rtd_map_;
  std::shared_ptr<RtmClient> rtm_client_;
};

}  // namespace rtmonitor

#endif  // RTMONITOR__RTMONITOR_HPP_
