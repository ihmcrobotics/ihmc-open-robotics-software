byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

byte robot_side 255

float64 desired_position
bool torque_on
float64 torque_limit
float64 measured_position
float64 measured_torque
int32 measured_temperature
float64 measured_velocity
int32 error_codes
int32 realtime_tick

bool is_calibrated
bool needs_reset