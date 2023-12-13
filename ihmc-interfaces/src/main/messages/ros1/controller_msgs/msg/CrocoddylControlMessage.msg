# This is a message that holds data to describe the control.
# 
# The control is defined by:
# * the type of control input,
# * the type of control parametrization,
# * the control vector, and
# * the feedback gain
# 
# This information allow us to compute the control in a desired time.
# The order of the control is designed to be consistent with the URDF.

# Type of control inputs
uint8 EFFORT=0

uint8 ACCELERATION_CONTACTFORCE=1

# Type of control parametrization
uint8 POLYZERO=0

uint8 POLYONE=1

uint8 POLYTWO=2

uint8 input

uint8 parametrization

# This represents the control input vector
float64[] u

# This represents the feedback gain
controller_msgs/CrocoddylFeedbackGainMessage gain


