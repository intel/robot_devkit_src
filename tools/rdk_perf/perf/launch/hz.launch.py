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

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='rdk_perf', 
            node_executable='hz',
            node_name='instance1',
            parameters=[{'monitored_node':'camera'},
                        {'monitored_topic':'/camera/color/image_raw'},
                        {'window_size':'10000'}],
            remappings=[('/hz','/perf/hz1')],
            output='screen'),
# Support multiple topics monitoring. 
# Add other nodes as follows with different node_name and output topic name.

#        launch_ros.actions.Node(
#            package='rdk_perf', 
#            node_executable='hz',
#            node_name='instance2',
#            parameters=[{'monitored_node':'camera'},
#                        {'monitored_topic':'/camera/infra1/image_rect_raw'},
#                        {'window_size':'10000'}],
#            remappings=[('/hz','/perf/hz2')],
#            output='screen'),
        launch_ros.actions.Node(
            package='rqt_plot', 
            node_executable='rqt_plot',
            output='screen'),
    ])
