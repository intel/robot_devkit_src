std_msgs/Header header              # timestamp in header is the time when msg was dispatched
string topic                        # Name of the topic monitored
bool pub                            # True if time captured at publisher end of the topic, false if at subscription end
int32 rate                          # Desired rate
int32 jitter                        # Acceptable jitter in percentage
int32 iteration                     # Iteration count
uint64 looptime                     # Looptime value in nanosecs
