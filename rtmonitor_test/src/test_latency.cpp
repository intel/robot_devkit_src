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

#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rtmonitor/rtmonitor.hpp"
#include "rtmonitor_msgs/msg/loop_time.hpp"

using namespace std::chrono_literals;

class Producer : public rclcpp::Node
{
public:
  explicit Producer(const std::string & topic_name)
  : Node("producer")
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<rtmonitor_msgs::msg::LoopTime>(topic_name, qos);

    timer_ = this->create_wall_timer(100ms, std::bind(&Producer::produce_message, this));
  }
  ~Producer() {}
  void produce_message()
  {
    rtmonitor_msgs::msg::LoopTime::UniquePtr msg(new rtmonitor_msgs::msg::LoopTime());
    msg->header.stamp = this->now();
    msg->iteration = count_++;
    RCLCPP_INFO(this->get_logger(), "Producer: [%d]", msg->iteration);
    pub_->publish(std::move(msg));
  }

private:
  size_t count_ = 1;
  rclcpp::Publisher<rtmonitor_msgs::msg::LoopTime>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class Consumer : public rclcpp::Node
{
public:
  explicit Consumer(const std::string & topic_name)
  : Node("consumer")
  {
    sub_ =
      create_subscription<rtmonitor_msgs::msg::LoopTime>(topic_name, 10,
        std::bind(&Consumer::consume_message, this, std::placeholders::_1));

    rtm_.init("msg_lat");
  }
  ~Consumer() {}
  void consume_message(const rtmonitor_msgs::msg::LoopTime::UniquePtr msg)
  {
    // Measure latency between publish and received
    rtm_.calc_latency("msg_lat", msg->header.stamp, this->now());
    RCLCPP_INFO(this->get_logger(), "Consumer: [%d]", msg->iteration);
  }

private:
  rclcpp::Subscription<rtmonitor_msgs::msg::LoopTime>::SharedPtr sub_;
  rtmonitor::RealTimeMonitor rtm_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto producer = std::make_shared<Producer>("rt_monitor");
  auto consumer = std::make_shared<Consumer>("rt_monitor");

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(producer);
  executor.add_node(consumer);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
