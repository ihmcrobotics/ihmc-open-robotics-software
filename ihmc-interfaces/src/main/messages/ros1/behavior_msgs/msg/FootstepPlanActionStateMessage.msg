
int8 TRIGGERED=0

int8 FOOTSTEP_PLANNING=1

int8 PLANNING_FAILED=2

int8 PLANNING_SUCCEEDED=3

int8 PLAN_COMMANDED=4

int8 PLAN_EXECUTION_COMPLETE=5

# Parent state fields
behavior_msgs/ActionNodeStateMessage state

# Definition
behavior_msgs/FootstepPlanActionDefinitionMessage definition

# The footsteps, with a maximum of 50
behavior_msgs/FootstepPlanActionFootstepStateMessage[] footsteps

# Execution state
int8 execution_state

# Total number of footsteps; used for walking actions
uint16 total_number_of_footsteps

# Incomplete footsteps; used for walking actions
uint16 number_of_incomplete_footsteps

# Desired left footsteps
ihmc_common_msgs/SE3TrajectoryPointMessage[] desired_left_footsteps

# Desired right footsteps
ihmc_common_msgs/SE3TrajectoryPointMessage[] desired_right_footsteps

# Current left pose
geometry_msgs/Pose current_left_foot_pose

# Current right pose
geometry_msgs/Pose current_right_foot_pose


