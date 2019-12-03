# This message is part of the IHMC whole-body controller API.
# This message contains joint-level desired values that are output from the whole-body controller

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

string joint_name

byte CONTROL_MODE_POSITION = 0
byte CONTROL_VELOCITY = 1
byte CONTROL_EFFORT = 2
byte CONTROL_DISABLED = 3

byte control_mode 255

bool has_desired_torque
bool has_desired_position
bool has_desired_velocity
bool has_desired_acceleration
bool has_stiffness
bool has_damping
bool has_master_gain
bool has_velocity_scaling
bool has_position_integration_break_frequency
bool has_velocity_integration_break_frequency
bool has_position_integration_max_error
bool has_velocity_integration_max_error
bool has_position_feedback_max_error
bool has_velocity_feedback_max_error

float64 desired_torque
float64 desired_position
float64 desired_velocity
float64 desired_acceleration
float64 stiffness
float64 damping
float64 master_gain
float64 velocity_scaling
float64 position_integration_break_frequency
float64 velocity_integration_break_frequency
float64 position_integration_max_error
float64 velocity_integration_max_error
float64 position_feedback_max_error
float64 velocity_feedback_max_error