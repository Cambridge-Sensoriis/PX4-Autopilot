# LANDING_TARGET MAVLink message Report

uint64 timestamp	# [us] Time since system start

uint8 MAV_FRAME_LOCAL_NED = 1	# absolute target position, obliges the sender to know where the vehicle is
uint8 MAV_FRAME_LOCAL_FRD = 20	# target position relative to the vehicle, in its heading frame

uint8 frame          	# MAV_FRAME. Angles are conventionally MAV_FRAME_BODY_FRD and the frame is not
                     	# checked for them. Positions are either MAV_FRAME_LOCAL_NED (absolute) or
                     	# MAV_FRAME_LOCAL_FRD (relative to the vehicle).
                     	#
                     	# MAV_FRAME_LOCAL_FRD is taken to mean: origin at the vehicle's body origin,
                     	# axes forward-right-down, rotated by heading only so roll and pitch are zero.
                     	# That is the ArduPilot convention for this value and what landing target
                     	# senders emit in practice. It is NOT the strict MAVLink definition, which
                     	# puts the origin at a fixed point on the earth and aligns the axes to the
                     	# heading at the moment the frame was created. Do not "correct" it to the
                     	# spec reading: real senders would break.
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
