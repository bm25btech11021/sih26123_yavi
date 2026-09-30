std_msgs/Header header
string robot_id
string[] assigned_bundle
string current_task_id
string current_phase
geometry_msgs/Point current_goal
geometry_msgs/Point[] horizon_path
geometry_msgs/Point[] execution_path
uint32 horizon_steps
uint32 execution_window
uint32 replan_count
float64 planning_latency_ms
bool is_valid

