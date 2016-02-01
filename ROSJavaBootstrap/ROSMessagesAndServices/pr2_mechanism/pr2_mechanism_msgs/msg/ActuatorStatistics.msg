# This message contains the state of an actuator on the pr2 robot.
# An actuator contains a motor and an encoder, and is connected
# to a joint by a transmission

# the name of the actuator
string name

# the sequence number of the MCB in the ethercat chain. 
# the first device in the chain gets deviced_id zero
int32 device_id

# the time at which this actuator state was measured
time timestamp

# the encoder position, represented by the number of encoder ticks
int32 encoder_count

# The angular offset (in radians) that is added to the encoder reading, 
# to get to the position of the actuator. This number is computed when the referece
# sensor is triggered during the calibration phase
float64 encoder_offset

# the encoder position in radians
float64 position

# the encoder velocity in encoder ticks per second
float64 encoder_velocity

# the encoder velocity in radians per second
float64 velocity

# the value of the calibration reading: low (false) or high (true)
bool calibration_reading

# bool to indicate if the joint already triggered the rising/falling edge of the reference sensor
bool calibration_rising_edge_valid
bool calibration_falling_edge_valid

# the encoder position when the last rising/falling edge was observed. 
# only read this value when the calibration_rising/falling_edge_valid is true
float64 last_calibration_rising_edge
float64 last_calibration_falling_edge

# flag to indicate if this actuator is enabled or not. 
# An actuator can only be commanded when it is enabled.
bool is_enabled

# indicates if the motor is halted. A motor can be halted because of a voltage or communication problem
bool halted

# the last current/effort command that was requested
float64 last_commanded_current
float64 last_commanded_effort

# the last current/effort command that was executed by the actuator
float64 last_executed_current
float64 last_executed_effort

# the last current/effort that was measured by the actuator
float64 last_measured_current
float64 last_measured_effort

# the motor voltate
float64 motor_voltage

# the number of detected encoder problems 
int32 num_encoder_errors

