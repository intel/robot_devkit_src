# Real Time Monitor
RTMonitor is a profiling tool built to monitor and capture real time performance metrics of ROS2 C++ Application.

## 1. Features
* Simple APIs for code instrumentation
* Measure performance metrics like looptime, latency, elapsed time etc
* Notify application on missed deadlines

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
```console
$ cd ~/rtmonitor_ws
$ source install/local_setup.bash
$ ros2 run rtmonitor_test test_looptime
```
### 3.4 Analyze
Log files will be generated in the /tmp directory. Python script can be used to create a plot of timing metrics to analyze
```console
$ cd ~/rtmonitor_ws/src/rtmonitor/rtmonitor_tools
$ python plot_bar.py -f /tmp/log_producer.txt
```


## 4. Using RTMonitor to profile ROS2 Project
The package `rtmonitor` can be integrated with other ROS2 packages to profile the performance of those packages
### 4.1 Build system integration
#### 4.1.1 package.xml
Add `rtmonitor` and `rtmonitor_msgs` as dependencies in your ROS2 package.xml file. Ignore `rtmonitor_msgs` if not needed.
```xml
<package format="2">
  + <build_depend>rtmonitor</build_depend>
  + <build_depend>rtmonitor_msgs</build_depend>
  
  + <exec_depend>rtmonitor</exec_depend>
  + <exec_depend>rtmonitor_msgs</exec_depend>
</package>
```
#### 4.1.2 CMakeLists.txt
Add `rtmonitor` and `rtmonitor_msgs` as dependencies in your ROS2 package CMakeLists.txt file. Ignore `rtmonitor_msgs` if not needed.
```cmake
# find dependencies
+ find_package(rtmonitor REQUIRED)
+ find_package(rtmonitor_msgs REQUIRED)

ament_target_dependencies(<target>
  "rclcpp"
+  "rtmonitor"
+  "rtmonitor_msgs"
  "std_msgs")

```
### 4.2 Instrumenting code
`rtmonitor` requires explicit code instrumentation. The library provides APIs to initialize and calculate metrics
#### 4.2.1 Initialize
The real-time events (update loop, message pub/sub, function time) that are to be monitored and logged will first need to be registered and initialized. A unique string is used as an identifier for that particular event. The timing metrics of each event is captured separately in a different files.

Example : [test_looptime.cpp](rtmonitor_test/src/test_looptime.cpp#L38)
```cpp
  explicit Producer(const std::string & topic_name)
  : Node("producer")
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, qos);

    timer_ = this->create_wall_timer(100ms, std::bind(&Producer::produce_message, this));

    /* Initialize & Register callback*/
    rtm_.init("producer", 10 /*rate*/, 5 /*margin %age*/,
      std::bind(&Producer::cbLooptimeOverrun, this,
      std::placeholders::_1, std::placeholders::_2));
  }
```
It is possible to initialize without registering callback if not required. In this case, timing metric will be logged but no notification will be issued when deadlines missed

Example : [test_latency.cpp](rtmonitor_test/src/test_latency.cpp#L64)
```cpp
  explicit Consumer(const std::string & topic_name)
  : Node("consumer")
  {
    sub_ =
      create_subscription<rtmonitor_msgs::msg::LoopTime>(topic_name, 10,
        std::bind(&Consumer::consume_message, this, std::placeholders::_1));

    /* Initialize */
    rtm_.init("msg_lat");
  }
```
#### 4.2.2 Calculate metrics
To calculate and log the timing metrics, specific APIs along with string identifier needs to be called

##### 4.2.2.1 Loop Time
For an event happening periodically in a loop, looptime is the time period between two successive occurence of such event. `rtmonitor` can measure looptime and provide metrics to evaluate the jitter. Also, it is possible for the application to request for a callback from `rtmonitor` whenever jitter is beyond acceptable margin and deadline is missed.

Example: [test_looptime.cpp](rtmonitor_test/src/test_looptime.cpp)
```cpp
  void consume_message(const std_msgs::msg::String::UniquePtr msg)
  {
    // Measure looptime
    rtm_.calc_looptime("consumer", this->now());
    RCLCPP_INFO(this->get_logger(), "Consumer: [%s]", msg->data.c_str());
  }

```

##### 4.2.2.2 ROS2 Message Latency
There is a latency between ROS2 message published and when a callback is received on the subscriber side. `rtmonitor` can calculate this latency as difference of the timestamp on the message header and the time when message is received by the subscriber.

Example: [test_latency.cpp](rtmonitor_test/src/test_latency.cpp)
```cpp
  void consume_message(const rtmonitor_msgs::msg::LoopTime::UniquePtr msg)
  {
    // Measure latency between publish and received
    rtm_.calc_latency("msg_lat", msg->header.stamp, this->now());
    RCLCPP_INFO(this->get_logger(), "Consumer: [%d]", msg->iteration);
  }

```

##### 4.2.2.3 Elapsed Time
Elapsed time is the time it takes to get from one specified point in code to another.

Example: [test_elapsed.cpp](rtmonitor_test/src/test_elapsed.cpp)
```cpp
  void do_something_intensive()
  {
    // Start timer
    rtm_.calc_elapsed("something_intensive", true, this->now());
    usleep(100000);
    // Stop timer
    rtm_.calc_elapsed("something_intensive", false, this->now());
  }

```

### 4.3 Collecting Data
All the timing metrics generated is saved in a log file in the /tmp directory. The naming convention of the log file is log_<string-id>.txt . A separate log file is generated for every event initialized.

### 4.4 Analyzing Data
The timing metrics can be analyzed by post-processing the log files generated to create a plot out of it. This can be done by a python script [plot_bar.py](rtmonitor_tools/plot_bar.py) provided

```console
$ cd ~/rtmonitor_ws/src/rtmonitor/rtmonitor_tools
$ python plot_bar.py -f /tmp/log_producer.txt
```

## 5. Contributing
Feel free to submit Pull Requests.

## 6. Reporting issue
If run into any issue or wish to request for a feature, feel free to report issue in this project.
