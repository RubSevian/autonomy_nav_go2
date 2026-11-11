# One atomic command/readiness envelope. Time is host monotonic ns, NOT ROS time.
uint32 schema_version 1
uint64 observed_ns
string<=128 session_id
bool navigation_ready
bool perception_ready
bool command_enabled
float64[3] command
float64 command_age_s
