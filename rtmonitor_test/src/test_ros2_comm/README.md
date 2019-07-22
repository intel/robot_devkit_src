# Test ROS2 Communication
test_ros2_comm is a tool built to test and capture performance metrics of ROS2 Communication.

## 1. Features
* Measure performance metrics of ROS2.0 Communication with different configurations
* Generate data for visualization and post processing.

## 2. Metrics
* **Round Trip Time (Latency)**: Time taken by a ROS2 message to travel from client to server and from server back to client.
* **Jitter**: Amount of variation in round trip time. To calculate jitter, take the difference in RTT between samples, add them, and then divide by the number of samples (minus 1)
* **Reliability**: Ability of the message to reach their destination without getting lost or corrupted. To calculate reliability, take the number of messages received successfully and divide by number of samples sent.
* **Memory Usage**: Dynamic memory allocated at run time
* **CPU Usage**: CPU consumed at run time

## 3. Configurations
* Publisher and Subscriber distribution
  * Ping-Pong: Sender and Receiver in two separate systems connected over WiFi 	
  * Inter: Sender and Receiver in two separate process running in one single system	
  * Intra: Sender and Receiver running in same Process	

## 2. Dependencies
* ROS2

## 3. Getting Started
### 3.1 Fetch
```console
$ mkdir -p ~/rtmonitor_ws/src
$ cd ~/rtmonitor_ws/src
$ git clone <rtmonitor>

```
### 3.2 Build
```console
$ source ~/ros2_ws/install/local_setup.bash
$ cd ~/rtmonitor_ws
$ colcon build --symlink-install

```
### 3.3 Test
Test ROS2 communication between two systems with default configurations
```console
In System 1
$ source ~/ros2_ws/install/local_setup.bash
$ cd ~/rtmonitor_ws
$ source ./install/local_setup.bash
$ ros2 run rtmonitor_test test_ros2_comm -m ping

In System 2
$ source ~/ros2_ws/install/local_setup.bash
$ cd ~/rtmonitor_ws
$ source ./install/local_setup.bash
$ ros2 run rtmonitor_test test_ros2_comm -m pong
```
### 3.4 Analyze
Log files will be generated in the /tmp directory. Python script can be used to create a plot of timing metrics to analyze
```console
$ cd ~/rtmonitor_ws/src/rtmonitor/rtmonitor_tools
$ python plot_latency.py -f /tmp/log_msg_RTT.txt
```

