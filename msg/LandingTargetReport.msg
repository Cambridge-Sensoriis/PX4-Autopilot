# LANDING_TARGET MAVLink message Report

uint64 timestamp	# [us] Time since system start
uint8 frame          	# MAV_FRAME (MAV_FRAME_BODY_FRD=12 for angles, MAV_FRAME_LOCAL_NED=0 for position)
uint8 target_num     	# target ID
uint8 type           	# LANDING_TARGET_TYPE enum


# Angle-based (when position_valid == false)
float32 angle_x      	# tan(theta_x), same semantics as IrlockReport.pos_x
float32 angle_y
float32 size_x
float32 size_y
float32 distance     	# NaN if unknown

# Position-based (when position_valid == true)
bool position_valid
float32 pos_x        	# in given frame (m)
float32 pos_y
float32 pos_z

# Sensor orientation in vehicle body frame (w, x, y, z order; zero-rotation is 1, 0, 0, 0)
# Set to [0, 0, 0, 0] if unknown/not provided
float32[4] q
