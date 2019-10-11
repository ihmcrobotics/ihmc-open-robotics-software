int8 CONTROL_MODE_POSITION=0

int8 CONTROL_VELOCITY=1

int8 CONTROL_EFFORT=2

int8 CONTROL_DISABLED=3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 255
int8 control_mode

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


