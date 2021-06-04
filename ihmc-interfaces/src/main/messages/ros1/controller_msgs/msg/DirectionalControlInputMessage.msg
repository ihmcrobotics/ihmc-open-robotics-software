# This message is sent to command the robot to move as if controlled by a joystick.
# The forward, right and clockwise parameters are a direction of travel and speed
# represented as a set of 3 movement impulses:
# - forwards/backwards
# - left/right
# - clockwise/counter-clockwise rotation
# 
# Each impulse value is a floating point number, v, with -1 <= v <= 1, where
# v<0 indicates backwards/left/counter-clockwise movement at (-v)*max speed,
# v=0 indicates no change,
# v>0 indicates forwards/right/clockwise at v*max speed.

uint32 sequence_id

float64 forward

float64 right

float64 clockwise


