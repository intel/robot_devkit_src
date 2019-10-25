# rdk_perf

## Introduction  
  Performance tool for ROS2 topics and show result in rqt at runtime.

## Build  
  ```
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws/src
  git clone https://gitlab.devtools.intel.com/otc-rse/rdk_perf
  cd ~/ros2_ws
  colcon build --symlink-install
  source install/local_setup.bash
  ```
## Run  
  * Adjust the parameters in launch file before running. Please refer [this section](#jump) for details.  
    * Delay  
    ```
    ros2 launch rdk_perf delay.launch.py
    ``` 
    * HZ
    ```
    ros2 launch rdk_perf hz.launch.py
    ```
  * Show result in rqt_plot  
  
  ![pic](https://github.com/intel/robot_devkit_src/blob/master/tools/rdk_perf/doc/rqt_plot.png)
  
  * Monitor multiple topics at the same time  
  For example, add another node in [delay.launch.py](https://github.com/intel/robot_devkit_src/blob/master/tools/rdk_perf/perf/launch/delay.launch.py) as follows:
  ```
          launch_ros.actions.Node(
            package='rdk_perf',
            node_executable='delay',
            node_name='instance2',
            parameters=[{'monitored_node':'your_node'},
                        {'monitored_topic':'your_topic'},
                        {'window_size':'10000'}],
            remappings=[('/delay','/perf/delay2')],
            output='screen'),
  ```
## <span id="jump">Parameters in launch file</span>
  * monitored_node  
    The node name need to be monitored
  * monitored_topic  
    The topic name need to be monitored
  * window_size  
    The number of the samples is used to analyze
