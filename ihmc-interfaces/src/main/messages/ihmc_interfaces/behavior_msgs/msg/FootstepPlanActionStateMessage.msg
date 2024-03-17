byte FOOTSTEP_PLANNING = 0
byte PLANNING_FAILED = 1
byte PLANNING_SUCCEEDED = 2
byte PLAN_COMMANDED = 3

# Parent state fields
behavior_msgs/ActionNodeStateMessage state

# Definition
behavior_msgs/FootstepPlanActionDefinitionMessage definition

# The footsteps, with a maximum of 50
behavior_msgs/FootstepPlanActionFootstepStateMessage[<=50] footsteps

# Goal to parent frame Z
float64 goal_to_parent_z

# Goal to parent frame Pitch
float64 goal_to_parent_pitch

# Goal to parent frame Roll
float64 goal_to_parent_roll

# Execution state
byte execution_state

# Total number of footsteps; used for walking actions
uint16 total_number_of_footsteps

# Incomplete footsteps; used for walking actions
uint16 number_of_incomplete_footsteps

# Desired left footsteps
ihmc_common_msgs/SE3TrajectoryPointMessage[<=50] desired_left_footsteps

# Desired right footsteps
ihmc_common_msgs/SE3TrajectoryPointMessage[<=50] desired_right_footsteps

# Current left pose
geometry_msgs/Pose current_left_foot_pose

# Current right pose
geometry_msgs/Pose current_right_foot_pose
