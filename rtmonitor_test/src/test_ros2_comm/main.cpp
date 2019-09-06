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

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "rtm_pinger.hpp"
#include "rtm_ponger.hpp"

#define MODE_PING 0
#define MODE_PONG 1

void print_usage()
{
  printf("Usage for test_ros2_comm app:\n");
  printf("test_ros2_comm [-m mode] [-t time-duration] [-f "
    "message-frequency][-q rmw-qos] [-d dds-type] [-n "
    "num-pubsub] [-h]\n");
  printf("options:\n");
  printf("-h print-help: Print this help function.\n");
  printf("-m test-mode: Specify the mode(ping/pong) in which to run the test. Defaults to "
    "ping.\n");
  printf("-t test-time: Time duration of the test in mins. Defaults to 2 mins .\n");
  printf("-l msg-length: Size of message to be published. Defaults to 0 (empty message) .\n");
  printf("-r pub-rate: Rate(Frequency) at which message is to be published. Defaults to 10Hz.\n");
  printf("-d dds-type: Type of DDS to be used. Defaults to FastRTPS .\n");
  printf("-q rmw-qos: QoS configuration to be used. Defaults to 0 .\n"
    "\t \t DEFAULT: 0 \n"
    "\t \t KEEP_ALL: 1 \n"
    "\t \t KEEP_LAST: 2 \n"
    "\t \t RELIABLE: 3 \n"
    "\t \t BEST_EFFORT: 4 \n"
    "\t \t DURABILITY_VOLATILE: 5 \n"
    "\t \t TRANSIENT_LOCAL: 6 \n"
    "\t \t LIVELINESS: 7 \n"
    "\t \t LIFESPAN: 8 \n"
    "\t \t DEADLINE: 9 \n"
    "\t \t LARGE_DATA: 10 \n");
  printf("-p num-pub: Number of publishers . Defaults to 1 .\n");
  printf("-s num-sub: Number of subscribers. Defaults to 1 .\n");
  printf(
    "-n num-topic : Number of topics divided proportionately among pubs & subs. Defaults to 1 .\n");

}

int main(int argc, char * argv[])
{
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  // TODO(lbegani): Explore passing settings as config file
  // TODO(lbegani): Explore using env variables instead of cli options.
  // TODO(lbegani): Explore taking inputs from user at runtime.
  uint32_t mode_ = MODE_PING;
  uint32_t dur_ = 2;
  uint32_t rate_ = 10;
  uint32_t len_ = 0;
  uint32_t qos_ = QOS_DEFAULT;

  char * mode = rcutils_cli_get_option(argv, argv + argc, "-m");
  if (nullptr != mode) {
    printf("Mode = %s \n", mode);
    if (strcmp("ping", mode) == 0) {
      mode_ = MODE_PING;
    } else if (strcmp("pong", mode) == 0) {
      mode_ = MODE_PONG;
    }
  }

  if (rcutils_cli_option_exist(argv, argv + argc, "-t")) {
    dur_ = std::stoul(rcutils_cli_get_option(argv, argv + argc, "-t"));
  }

  if (rcutils_cli_option_exist(argv, argv + argc, "-q")) {
    qos_ = std::stoul(rcutils_cli_get_option(argv, argv + argc, "-q"));
  }

  if (rcutils_cli_option_exist(argv, argv + argc, "-r")) {
    rate_ = std::stoul(rcutils_cli_get_option(argv, argv + argc, "-r"));
  }

  if (rcutils_cli_option_exist(argv, argv + argc, "-l")) {
    len_ = std::stoul(rcutils_cli_get_option(argv, argv + argc, "-l"));
  }

  printf("Test Mode = %d\n", mode_);

  rclcpp::init(argc, argv);

  std::vector<std::string> topic_index_list;
  topic_index_list.push_back("1");


  if (mode_ == MODE_PING) {
    printf("Test Duration = %d \n", dur_);
    printf("QoS Configration = %d \n", qos_);
    printf("Publisher Rate = %d \n", rate_);
    printf("Message Length = %d \n", len_);
    // rclcpp::spin(std::make_shared<RtmPinger>());
    rclcpp::spin(std::make_shared<RtmPinger>(topic_index_list, dur_, qos_, rate_, len_));
  } else {
    printf("QoS Configration = %d \n", qos_);
    // rclcpp::spin(std::make_shared<RtmPonger>());
    rclcpp::spin(std::make_shared<RtmPonger>(topic_index_list, qos_));
  }

  rclcpp::shutdown();

  return 0;
}
