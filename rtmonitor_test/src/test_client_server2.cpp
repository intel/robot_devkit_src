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
#include "std_msgs/msg/string.hpp"
#include "rtmonitor/rtmonitor.hpp"

using namespace std::chrono_literals;

class Producer : public rclcpp::Node
{
public:
  explicit Producer(const std::string & topic_name)
  : Node("producer")
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, qos);

    timer_ = this->create_wall_timer(100ms, std::bind(&Producer::produce_message, this));

#if 0
    rtm_.init("producer", 10 /*rate*/, 5 /*margin %age*/,
      std::bind(&Producer::cbLooptimeOverrun, this,
      std::placeholders::_1, std::placeholders::_2));
#endif
  }
  ~Producer() {}
  void init()
  {
    rtm_.init(shared_from_this(), "producer2");
  }
  void produce_message()
  {
    // Measure looptime
    rtm_.calc_looptime("producer2", this->now());
    std_msgs::msg::String::UniquePtr msg(new std_msgs::msg::String());
    msg->data = "RT Message: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Producer: [%s]", msg->data.c_str());
    pub_->publish(std::move(msg));
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

  auto producer = std::make_shared<Producer>("rtm_prod2");
  producer->init();

  rclcpp::spin(producer);
  rclcpp::shutdown();
  return 0;
}
