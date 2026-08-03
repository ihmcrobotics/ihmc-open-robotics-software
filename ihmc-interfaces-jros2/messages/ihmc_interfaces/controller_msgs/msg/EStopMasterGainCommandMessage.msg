# A message for sending necessary startup, shutdown, and high-level operational command variables to the robot

# Auto startup and shutdown
bool request_startup
bool request_shutdown
bool execute_startup_shutdown

# SOFT-E-STOP to set
bool estop
bool execute_estop

# AvatarLowLevelOutputProcessor masterGain variable to set
float64 desired_master_gain
bool execute_master_gain

# True if this is a request to servo the robot
# This is used to enable and ramp up/down the robot's controller gains, to go into freeze or stand prep
bool servo_robot
bool execute_servo_robot

# True if this is a request to immediately zero the gains which can be violent and dangerous
bool unservo_quickly
bool execute_unservo_quickly

# Enable publishing of ROS commands to the robot
bool enable_publishing_to_robot
bool execute_enable_publishing_to_robot

# Clear robot faults
bool clear_faults
bool execute_clear_faults

# Calibrate robot
bool calibrate_robot
bool execute_calibrate_robot

# Enable all actuators
bool enable_actuators
bool execute_enable_actuators