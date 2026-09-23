# Atomic planning outcome; non-success results contain no segments or trajectory points.
# This is an obstacle-point map planner, not a proof of observed free space or flight clearance.
# Partial prefix; endpoint need not be the goal or have zero velocity.
uint8 REACH_HORIZON=1
# Curve reaches requested goal position with the wrapper's zero terminal velocity.
uint8 REACH_END=2
uint8 NO_PATH=3
# Reserved upstream status; the current implementation does not emit it.
uint8 NEAR_END=4
uint8 INVALID_INPUT=5
uint8 NO_MAP=6
uint8 TIMEOUT=7
uint8 NODE_LIMIT=8
uint8 WAITING_FOR_INPUT=9
uint8 STALE_INPUT=10
uint8 FRAME_MISMATCH=11
# Planning-attempt start time in the ROS clock and the common world frame.
std_msgs/Header header
# Per-process counters; both restart from zero when the node restarts (not global session IDs).
uint64 sequence
uint64 goal_revision
# Original timestamps of the last accepted inputs; may use a device clock in receive mode.
# On input failure these may still describe older accepted data; inspect status first.
builtin_interfaces/Time cloud_stamp
builtin_interfaces/Time odometry_stamp
# Only REACH_END and REACH_HORIZON carry trajectories. detail is diagnostic text, not an API enum.
uint8 status
string detail
# Search wall time in milliseconds; excludes map building/output conversion. Zero if search was skipped.
float64 planning_ms
# Sum of filtered points in both map banks, possibly including cross-bank duplicates.
uint64 map_points
# Exact, ordered polynomial segments; each segment uses its own local time starting at zero.
PolynomialSegment[] segments
# Samples of those segments; time_from_start is relative to the curve start, including exact endpoints.
# One translation joint named "position"; orientation is identity and angular derivatives are zero.
# Segment-boundary acceleration is taken from the preceding segment.
trajectory_msgs/MultiDOFJointTrajectory trajectory
