# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When false, the toolbox will only publish a status with KinematicsToolboxOutputStatus such that the user can validate
# that the solver is working properly.
# When true, the toolbox will stream to the IHMC walking controller the desired robot configuration. The status mentioned is
# still published so the user can compare the solver desired state against the actual robot state.
bool stream_to_controller

# When starting to stream to controller, a blending is initiated over a fixed duration so the IHMC walking controller smoothly
# reaches for the current IK configuration.
# A larger value will result in a smoother initial transition, while a shorter duration will result in a quicker response when starting to stream.
# Set to <= 0.0 to use the default value.
# Field default value -1.0
float64 stream_initial_blend_duration

# Constraint on the maximum angular velocity resulting from any user inputs.
# A lower value will reduce the speed at which the robot can move, while a higher value will improve response.
# Set to <= 0.0 to use the default value.
# Field default value -1.0
float64 angular_rate_limitation

# Constraint on the maximum linear velocity resulting from any user inputs.
# A lower value will reduce the speed at which the robot can move, while a higher value will improve response.
# Set to <= 0.0 to use the default value.
# Field default value -1.0
float64 linear_rate_limitation

# The list of inputs the solver is to be tracking.
# When streaming inputs from a VR UI environment, it is convenient to use the fields control_frame_position_in_end_effector and
# control_frame_orientation_in_end_effector from KinematicsToolboxRigidBodyMessage to adjust the user's controllers with respect
# to the robot's end-effectors.
controller_msgs/KinematicsToolboxRigidBodyMessage[] inputs


