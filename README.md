# Real Time Monitor
RTMonitor is a profiling tool built to monitor and capture real time performance metrics of ROS2 C++ Application.

## 1. Features
* Simple APIs for code instrumentation.
* Calculate & log performance metrics like looptime, latency, elapsed time etc.
* Notify application on missed deadlines.
* Scripts to create plot from logged data.
* Publish the performance metrics data on ROS2 topic

## 2. Dependencies
* Ubuntu 18.04
* ROS2 Dashing

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
`rtmonitor` requires explicit code instrumentation. The library provides APIs to initialize and calculate metrics. The timing data of each metric is captured separately in a different files.
#### 4.2.1 Initialize
`rtmonitor` requires initialization by ROS2 application for two conditions where -
* App needs to calculate elapsed time between events not in a single process. This means using `calc_elapsed_g` API.
* App needs to publish the performance metric data on ROS2 topic.

For conditions other than the two mentioned above, initialization is not required. The metrics will get registered implicitly on any first call with identifier.

Example : [test_publish_metric.cpp](rtmonitor_test/src/test_publish_metric.cpp#L30)
```cpp
  explicit Producer(const std::string & topic_name)
  : Node("producer")
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, qos);

    timer_ = this->create_wall_timer(100ms, std::bind(&Producer::produce_message, this));
  }
  ~Producer() {}
  void init()
  {
    rtm_.init(shared_from_this());
  }

```

#### 4.2.2 Register callback
It is possible to receive callback for a metric, whenever a deadline is missed. App can set the expected duration for the metric and acceptable jitter. Any deviation more than the acceptable jitter will invoke the callback.

Example: [test_elapsed.cpp](rtmonitor_test/src/test_elapsed.cpp#49)
```cpp
  explicit Producer(const std::string & topic_name)
  : Node("producer")
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, qos);

    timer_ = this->create_wall_timer(100ms, std::bind(&Producer::produce_message, this));

    uint64_t exp_perf_ns = ACCEPTABLE_DURATION;
    uint64_t exp_jitter_ns = (exp_perf_ns/100)*JITTER_PERCENT;

    rtm_.register_callback(
      "producer", rclcpp::Duration(exp_perf_ns), rclcpp::Duration(exp_jitter_ns),
      std::bind(&Producer::cbLooptimeOverrun, this,
      std::placeholders::_1, std::placeholders::_2));
  }

```

#### 4.2.3 Calculate metrics
To calculate and log the timing metrics, specific APIs along with string identifier needs to be called. Metrics will get registered on the first call to calculate metrics.

##### 4.2.3.1 Loop Time
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

##### 4.2.3.2 ROS2 Message Latency
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

##### 4.2.3.3 Elapsed Time
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
##### 4.2.3.4 Elapsed Time Global
Elapsed time global is the time difference between two points in the code, where the two points can be within a single process or different process. The time information along with metric identifier is posted to a global ROS2 service running.
ROS2 service calculates the elapsed time and logs the data in `/tmp/log_rtm_service.txt`. Before using `calc_elapsed_g`, initialization of `rtmonitor` is required.

Example: [test_elapsed_global.cpp](rtmonitor_test/src/test_elapsed_global.cpp#L59)
```cpp
  void produce_message()
  {
    // Measure looptime
    rtm_.calc_looptime("producer", this->now());
    rtm_.calc_elapsed_g("produce_message", true, this->now());
    do_something_intensive();
    std_msgs::msg::String::UniquePtr msg(new std_msgs::msg::String());
    msg->data = "RT Message: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Producer: [%s]", msg->data.c_str());
    pub_->publish(std::move(msg));
    rtm_.calc_elapsed_g("produce_message", false, this->now());
  }

```

### 4.3 Collecting Data
All the timing metrics generated is saved in a log file in the /tmp directory. The naming convention of the log file is log_<string-id>.txt.
A separate log file is generated for every metric. But for all the metrics calculated using `calc_elapsed_g`, there is a single file `/tmp/log_rtm_service.txt`

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
