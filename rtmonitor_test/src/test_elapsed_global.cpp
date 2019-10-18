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

#include <unistd.h>

#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rtmonitor/rtmonitor.hpp"

using namespace std::chrono_literals;

// Rate at which the loop runs in Hz
#define RATE 10

// Acceptable Jitter margin in percentage
#define JITTER_PERCENT 5

class Producer : public rclcpp::Node
{
public:
  explicit Producer(const std::string & topic_name)
  : Node("producer")
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, qos);

    timer_ = this->create_wall_timer(100ms, std::bind(&Producer::produce_message, this));

    uint64_t exp_perf_ns = 1e9/RATE;
    uint64_t exp_jitter_ns = (exp_perf_ns/100)*JITTER_PERCENT;

    rtm_.register_callback(
      "producer", rclcpp::Duration(exp_perf_ns), rclcpp::Duration(exp_jitter_ns),
      std::bind(&Producer::cbLooptimeOverrun, this,
      std::placeholders::_1, std::placeholders::_2));
  }
  ~Producer() {}
  void init()
  {
    rtm_.init(shared_from_this());
  }
  void produce_message()
  {
    // Measure looptime
    rtm_.calc_looptime("producer", this->now());
    rtm_.calc_elapsed_g("produce_message", true, this->now());
    do_something_intensive();
    std_msgs::msg::String::UniquePtr msg(new std_msgs::msg::String());
    msg->data = "RT Message: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Producer: [%s]", msg->data.c_str());
    pub_->publish(std::move(msg));
    rtm_.calc_elapsed_g("produce_message", false, this->now());
  }
  void do_something_intensive()
  {
    rtm_.calc_elapsed_g("something_intensive", true, this->now());
    usleep(1000000/RATE);
    rtm_.calc_elapsed_g("something_intensive", false, this->now());
  }
  void cbLooptimeOverrun(int iter_num, rclcpp::Duration jitter)
  {
    printf("Missed Deadline : %d\n", iter_num);
    (void)jitter;
  }

private:
  size_t count_ = 1;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rtmonitor::RealTimeMonitor rtm_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto producer = std::make_shared<Producer>("rt_msg");
  producer->init();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(producer);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

