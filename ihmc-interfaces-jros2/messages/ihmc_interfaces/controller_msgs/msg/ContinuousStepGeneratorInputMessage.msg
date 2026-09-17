# Input message for the ContinuousStepGenerator.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# If true, start stepping. If false, stand in place
bool walk

# Desired velocity setpoints
float64 forward_velocity
float64 lateral_velocity
float64 turn_velocity

# If true, desired velocity setpoints are normalized from a range of [-1.0, 1.0]
# If false, desired velocity setpoints are sent in m/s
bool are_velocities_normalized