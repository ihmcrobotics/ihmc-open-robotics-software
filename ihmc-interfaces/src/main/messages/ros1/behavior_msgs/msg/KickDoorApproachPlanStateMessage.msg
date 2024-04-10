
int8 FOOTSTEP_PLANNING=0

int8 PLANNING_FAILED=1

int8 PLANNING_SUCCEEDED=2

int8 PLAN_COMMANDED=3

# Parent state fields
behavior_msgs/ActionNodeStateMessage state

# Definition
behavior_msgs/KickDoorApproachPlanDefinitionMessage definition

# The footsteps, with a maximum of 50
behavior_msgs/FootstepPlanActionFootstepStateMessage[] footsteps

# Left foot goal pose
geometry_msgs/Pose left_foot_goal_pose

# Right foot goal pose
geometry_msgs/Pose right_foot_goal_pose

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


