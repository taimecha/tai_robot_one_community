# The frame_id is the RGB optical source frame. Horizontal scalar axes below
# are defined at the camera lens after applying the configured camera pitch:
# forward is parallel to the floor in the robot forward direction; lateral is
# positive to the robot's left. Camera-to-robot translation is NOT included.
std_msgs/Header header

# A consumer must require valid=true and a fresh header stamp. Invalid messages
# clear the target; status explains which measurement check failed.
bool valid
string status

# Front plane to camera lens along the horizontal forward axis, metres.
float32 camera_forward_m
# Same front plane to the chassis front midpoint, using a measured/configured
# camera lens offset. This is an estimate if the offset has not been taped.
float32 chassis_front_forward_m
# Opening midpoint relative to the camera lens; positive = left, metres.
float32 opening_lateral_m
# Pallet face angle: positive when LEFT leg is farther away than RIGHT, radians.
float32 face_yaw_rad
# Forward insertion direction relative to camera-forward: -face_yaw_rad.
# Positive means turn left; this assumes the two sampled legs share a plane.
float32 entry_heading_rad
float32 opening_width_m
# Vision estimate between lower beam and pallet feet, not floor-calibrated.
float32 opening_height_estimate_m
float32 leg_center_separation_m
float32 left_forward_m
float32 right_forward_m
float32 yolo_confidence
float32 rgb_depth_delta_ms
uint32 left_depth_samples
uint32 right_depth_samples
