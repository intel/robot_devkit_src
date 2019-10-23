# rdk_perf

* Introduction  
  Performance tool for ROS2 topics and show result in rqt.

* Build  
  ```
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws/src
  git clone https://gitlab.devtools.intel.com/otc-rse/rdk_perf
  cd ~/ros2_ws
  colcon build --symlink-install
  source install/local_setup.bash
  ```
* Run  
  Adjust the parameters in launch file before running. Please refer [this section](#jump) for details.
  * Delay  
  ```
  ros2 launch rdk_perf delay.launch.py
  ```
  * Hz  
  ```
  ros2 launch rdk_perf hz.launch.py
  ```
* <span id="jump">Parameters in launch file</span>
  * monitored_node  
    The node name need to be monitored
  * monitored_topic  
    The topic name need to be monitored
  * window_size  
    The number of the samples is used to analyze