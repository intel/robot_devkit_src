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
from collections import defaultdict
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from perf_msgs.msg import PerfValue

OUTPUT_TOPIC = '/hz'
DEFAULT_WINDOW_SIZE = 10000

class Hz(Node):

    def __init__(self):
        super().__init__(node_name='hz', namespace='/perf')
        self.lock = threading.Lock()
        self.last_printed_tn = 0
        self.msg_t0 = -1
        self.msg_tn = 0
        self.times = []
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

    def get_last_printed_tn(self):
        return self.last_printed_tn

    def set_last_printed_tn(self, value):
        self.last_printed_tn = value

    def get_msg_t0(self):
        return self.msg_t0

    def set_msg_t0(self, value):
        self.msg_t0 = value

    def get_msg_tn(self):
        return self.msg_tn

    def set_msg_tn(self, value):
        self.msg_tn = value

    def get_times(self):
        return self.times

    def set_times(self, value):
        self.times = value

    def callback(self, msg):
        with self.lock:
            curr_rostime = self._clock.now()
            if curr_rostime.nanoseconds == 0:
                if len(self.get_times()) > 0:
                    print('time has reset, resetting counters')
                    self.set_times([])
                return

            curr = curr_rostime.nanoseconds
            msg_t0 = self.get_msg_t0()
            if msg_t0 < 0 or msg_t0 > curr:
                self.set_msg_t0(curr)
                self.set_msg_tn(curr)
                self.set_times([])
            else:
                self.get_times().append(curr - self.get_msg_tn())
                self.set_msg_tn(curr)

            if len(self.get_times()) > self.window_size:
                self.get_times().pop(0)

            ret = self.get_hz()
            if ret == None:
                return
            rate, min_delta, max_delta, std_dev, window = ret
            # print('average rate: %.3f\n\tmin: %.3fs max: %.3fs std dev: %.5fs window: %s'
            #   % (rate * 1e9, min_delta * 1e-9, max_delta * 1e-9, std_dev * 1e-9, window))
            hz_msg = PerfValue()
            hz_msg.average = round(rate*1e9, 3)
            hz_msg.min = round(min_delta*1e-9, 3)
            hz_msg.max = round(max_delta*1e-9, 3)
            hz_msg.std_dev = round(std_dev*1e-9, 5)
            hz_msg.window = window
            self.pub.publish(hz_msg)
            

    def get_hz(self):
        """
        Calculate the average publising rate.
        :param topic: topic name, ``list`` of ``str``
        :returns: tuple of stat results
            (rate, min_delta, max_delta, standard deviation, window number)
            None when waiting for the first message or there is no new one
        """
        if not self.get_times():
            return
        elif self.get_last_printed_tn() == 0:
            self.set_last_printed_tn(self.get_msg_tn())
            return
        elif self.get_msg_tn() < self.get_last_printed_tn() + 1e9:
            return
  
        # Get frequency every one minute
        times = self.get_times()
        n = len(times)
        mean = sum(times) / n
        rate = 1. / mean if mean > 0. else 0

        # std dev
        std_dev = math.sqrt(sum((x - mean)**2 for x in times) / n)

        # min and max
        max_delta = max(times)
        min_delta = min(times)

        self.set_last_printed_tn(self.get_msg_tn())

        return rate, min_delta, max_delta, std_dev, n

def main(args=None):
    rclpy.init(args=args)

    node = Hz()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
