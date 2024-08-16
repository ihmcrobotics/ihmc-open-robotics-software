# This message is part of the IHMC whole-body controller API.
# This message enables a box manipulation state.

# If true, disables the bimanual manipulation manager and the fields below are ignored
bool disable

# Mass of the object being manipulated
# Field default value -1.0
float64 object_mass

# Lateral squeeze force while manipulating the object. Should approximately be squeeze_force = 0.5 * g * object_mass / friction_coefficient
# Field default value -1.0
float64 squeeze_force

# The squeeze and mass compensation forces ramp up over this duration (if negative a default value is used)
# Field default value -1.0
float64 initialize_duration

# Tracking error - if the distance between the hands varies from their initial distance by more than this value, the manipulation manager stops
# Field default value -1.0
float64 acceptable_tracking_error


