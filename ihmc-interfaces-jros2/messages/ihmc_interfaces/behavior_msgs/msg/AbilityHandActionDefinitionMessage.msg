# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Specifies the side of the robot that this message refers to.
byte robot_side 255

# AbilityHandManager.ControlMode ordinal
byte control_mode

# Grip to execute, if grip control mode
byte grip

# Goal positions for the joints
float32[6] goal_positions

# Goal positions for the joints
float32[6] goal_velocities

# Success criteria type (enum in AbilityHandActionDefinition)
byte success_criteria

# +/- tolerance in degrees for each joint position check
float32 each_joint_position_tolerance

# Sum all absolute joint movement into one value and check it exceeds this value
float32 sufficient_cumulative_joint_movement

# Enable a sinusoidal wiggling of joints if they are unresponsive
bool enable_wiggle_on_failure

# When to start wiggling
float32 time_to_wiggle

# Timeout regardless of modes, action will cease after this time
float32 ultimate_timeout
