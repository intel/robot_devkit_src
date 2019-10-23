# Copyright (c) 2019 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# The core computing method is from ros2topic package
# https://github.com/ros2/ros2cli/tree/master/ros2topic

import threading
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from perf_msgs.msg import PerfValue

OUTPUT_TOPIC = '/delay'
DEFAULT_WINDOW_SIZE = 10000

class Delay(Node):

    def __init__(self):
        super().__init__(node_name='delay', namespace='/perf')
        self.lock = threading.Lock()
        self.last_msg_tn = 0
        self.msg_t0 = -1.
        self.msg_tn = 0
        self.delays = []
        self._clock = self.get_clock()
        
        node_name = self.declare_parameter("monitored_node").get_parameter_value().string_value
        topic_name = self.declare_parameter('monitored_topic').get_parameter_value().string_value
        self.window_size = self.declare_parameter("window_size", DEFAULT_WINDOW_SIZE).get_parameter_value().integer_value
        self.pub = self.create_publisher(PerfValue, OUTPUT_TOPIC, 10)

        time.sleep(1)
        info = self.get_publisher_names_and_types_by_node(node_name, '')
        msg_type = ''
        for i in info:
            if i[0] == topic_name:
                msg_type = i[1][0]
            else:
                continue
        if msg_type == '':
            self.get_logger().info('The monitored topic is not available.')
        elif msg_type == 'sensor_msgs/msg/Image':
            self.sub = self.create_subscription(Image, topic_name, self.callback, qos_profile_sensor_data)  
        else:
            self.get_logger().info('Unsupported message type: %s' %msg_type)

    def callback(self, msg):
        if not hasattr(msg, 'header'):
            raise RuntimeError('msg does not have header')
        with self.lock:
            curr_rostime = self._clock.now()

            # time reset
            if curr_rostime.nanoseconds == 0:
                if len(self.delays) > 0:
                    print('time has reset, resetting counters')
                    self.delays = []
                return

            curr = curr_rostime.nanoseconds
            if self.msg_t0 < 0 or self.msg_t0 > curr:
                self.msg_t0 = curr
                self.msg_tn = curr
                self.delays = []
            else:
                # store the duration nanoseconds in self.delays
                duration = (curr_rostime - Time.from_msg(msg.header.stamp))
                self.delays.append(duration.nanoseconds)
                self.msg_tn = curr

            if len(self.delays) > self.window_size:
                self.delays.pop(0)

            if not self.delays:
               return
            ret = self.get_delay()
            if ret is None:
                print('no new messages')
                return
            delay, min_delta, max_delta, std_dev, window = ret
            # print('average delay: %.3f\n\tmin: %.3fs max: %.3fs std dev: %.5fs window: %s'
            #   % (delay * 1e-9, min_delta * 1e-9, max_delta * 1e-9, std_dev * 1e-9, window))
            delay_msg = PerfValue()
            delay_msg.average = round(delay*1e-9, 3)
            delay_msg.min = round(min_delta*1e-9, 3)
            delay_msg.max = round(max_delta*1e-9, 3)
            delay_msg.std_dev = round(std_dev*1e-9, 5)
            delay_msg.window = window
            self.pub.publish(delay_msg)

    def get_delay(self):
        """
        Calculate the average publising delay.
        :returns: tuple of stat results
            (rate, min_delta, max_delta, standard deviation, window number)
            None when waiting for the first message or there is no new one
        """
        if self.msg_tn == self.last_msg_tn:
            return
        if not self.delays:
            return
        n = len(self.delays)

        mean = sum(self.delays) / n
        std_dev = math.sqrt(sum((x - mean)**2 for x in self.delays) / n)

        max_delta = max(self.delays)
        min_delta = min(self.delays)

        self.last_msg_tn = self.msg_tn
        return mean, min_delta, max_delta, std_dev, n

def main(args=None):
    rclpy.init(args=args)
    node = Delay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    