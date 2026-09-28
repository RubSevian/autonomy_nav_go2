std_msgs/Header header

uint64 path_revision
bool requires_alignment
float32 lateral_clearance
float32 footprint_length
float32 footprint_width
float32 enter_yaw_tolerance
float32 continue_yaw_tolerance
float32 stop_yaw_tolerance
float32 recovery_yaw_limit
float32 speed_scale
# Point and lane tangent in the path's vehicle frame, selected by localPlanner
# before the narrow section. pathFollower converts/locks these exactly once.
float32 narrow_approach_x
float32 narrow_approach_y
float32 narrow_approach_heading
# Stable local-path identity and temporal-selection diagnostics.
int32 candidate_group_id
int32 candidate_rotation_id
int8 candidate_direction
bool candidate_narrow_mode
float32 current_score
float32 best_score
string switch_reason
string reason
