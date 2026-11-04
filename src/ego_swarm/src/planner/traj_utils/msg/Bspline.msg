int32 drone_id

int32 order
int64 traj_id
builtin_interfaces/Time start_time

float64[] knots
geometry_msgs/Point[] pos_pts

float64[] yaw_pts
float64 yaw_dt

# KR260 candidate ABI extension. Empty for legacy/non-KR260 use.
# Echo the accepted goal stamp so a controller cannot follow the previous goal.
string kr260_run_id
builtin_interfaces/Time kr260_goal_stamp

