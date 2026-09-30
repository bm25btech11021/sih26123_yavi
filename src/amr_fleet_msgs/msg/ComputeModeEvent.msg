# Event record published whenever an AMR transitions its adaptive compute mode.
std_msgs/Header header
string robot_id
string previous_mode
string current_mode
string trigger_signal
float64 trigger_value
float64 threshold_value
string reason
float64 dwell_time_sec

