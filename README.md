# Real Time Monitor
RTMonitor is a profiling tool built to monitor and capture real time performance metrics of ROS2 Application.

## 1. Features
* Simple APIs for code instrumentation
* Measure performance metrics like looptime, latency, elapsed time etc
* Notify application on missed deadlines

## 2. Dependencies
* ROS2

## 3. Getting Started
### Fetch
```
$ mkdir -p ~/rtmonitor_ws/src
$ cd ~/rtmonitor_ws/src
$ git clone <rtmonitor>

```
### Build
```
$ source ~/ros2_ws/install/local_setup.bash
$ cd ~/rtmonitor_ws
$ colcon build --symlink-install

```
### Test
```
$ cd ~/rtmonitor_ws
$ source install/local_setup.bash
$ ros2 run rtmonitor_test test_looptime
```
### Analyze
Log files will be generated in the /tmp directory. Python script can be used to create a plot of timing metrics to analyze
$ cd ~/rtmonitor_ws/src/rtmonitor/rtmonitor_tools
$ python plot_bar.py -f /tmp/log_producer.txt


## 4. Using RTMonitor to profile ROS2 Project
TODO
### Build system integration
TODO
### Instrumenting code
TODO
### Collecting Data
TODO
### Analyzing Data
TODO

## 5. Contributing
Feel free to submit Pull Requests.

## 6. Reporting issue
If run into any issue or wish to request for a feature, feel free to report issue in this project.
