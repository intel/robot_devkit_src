std_msgs/Header header              # timestamp in header is the time when msg was dispatched
string id                           # Name of the event monitored
bool is_start                       # True to indicate start of the event, false to indicate end
uint64 time_ns                      # Time at start/end of the event in nanosecs
