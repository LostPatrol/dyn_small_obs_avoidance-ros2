# Atomic planning outcome; non-success results contain no segments or trajectory points.
# This is an obstacle-point map planner, not a proof of observed free space or flight clearance.
uint8 REACH_HORIZON=1
uint8 REACH_END=2
uint8 NO_PATH=3
uint8 NEAR_END=4
uint8 INVALID_INPUT=5
uint8 NO_MAP=6
uint8 TIMEOUT=7
uint8 NODE_LIMIT=8
uint8 WAITING_FOR_INPUT=9
uint8 STALE_INPUT=10
uint8 FRAME_MISMATCH=11
std_msgs/Header header
uint64 sequence
uint64 goal_revision
builtin_interfaces/Time cloud_stamp
builtin_interfaces/Time odometry_stamp
uint8 status
string detail
float64 planning_ms
uint64 map_points
PolynomialSegment[] segments
trajectory_msgs/MultiDOFJointTrajectory trajectory
