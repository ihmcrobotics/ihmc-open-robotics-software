# This message is part of the IHMC quadruped controller API.
# This message sends the x gait settings used for determining gait.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte SLOW = 0
byte MEDIUM = 1
byte FAST = 2

float64 end_phase_shift -1.0
float64 stance_length -1.0
float64 stance_width -1.0
float64 step_ground_clearance -1.0
byte quadruped_speed

QuadrupedGaitTimingsPacket pace_slow_settings_packet
QuadrupedGaitTimingsPacket pace_medium_settings_packet
QuadrupedGaitTimingsPacket pace_fast_settings_packet
QuadrupedGaitTimingsPacket amble_slow_settings_packet
QuadrupedGaitTimingsPacket amble_medium_settings_packet
QuadrupedGaitTimingsPacket amble_fast_settings_packet
QuadrupedGaitTimingsPacket trot_slow_settings_packet
QuadrupedGaitTimingsPacket trot_medium_settings_packet
QuadrupedGaitTimingsPacket trot_fast_settings_packet
