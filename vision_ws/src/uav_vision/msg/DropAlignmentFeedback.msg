# Controller evaluation time in the mission frame (not the camera frame).
std_msgs/Header header
# Exact source image stamp from DropOffset.header.stamp; never a memory heartbeat.
time observation_stamp
time odom_stamp
# Same frozen action/context identity as AlignmentTargetContext.
string mission_id
uint32 decision_seq
uint16 attempt
uint8 payload_slot
uint32 semantic_target_id
time semantic_target_first_seen
string semantic_target_class
# Existing visual mode: drop_circle or drop_cross.
string align_mode
bool valid
# Controller owns compensated FC/outlet XY and the selected pre-descent gate.
bool aligned
# Vision loss must never clear the frozen goal. With reason=descent_timeout_ready,
# fresh frozen feedback authorizes the same-action descent timeout release lane.
bool frozen
geometry_msgs/Point target_fc
float64 horizontal_error_m
float64 horizontal_speed_mps
# descent_timeout_ready: controller's descent clock expired; not ALIGN age.
string reason
