
# Parent state fields
behavior_msgs/BehaviorTreeNodeStateMessage state

# The action's unique ID
uint32 id

# If the action is next for execution
bool is_next_for_execution

# If the action is to be executed concurrently
bool is_to_be_executed_concurrently

# If the node is able to execution
bool can_execute

# If the node is currently executing
bool is_executing

# If the node had a failure during it's last execution
bool failed

# Nominal execution duration
float64 nominal_execution_duration

# Time since execution started
float64 elapsed_execution_time

# Desired trajectory
ihmc_common_msgs/SE3TrajectoryPointMessage[] desired_trajectory

# Current pose
geometry_msgs/Pose current_pose

# Position distance to goal tolerance
float64 position_distance_to_goal_tolerance

# Orientation distance to goal tolerance
float64 orientation_distance_to_goal_tolerance


