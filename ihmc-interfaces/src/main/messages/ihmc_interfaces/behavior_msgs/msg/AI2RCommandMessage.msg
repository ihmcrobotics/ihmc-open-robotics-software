# Behavior to execute (checkpoint to jump to in the pre-loaded behavior collection)
string behavior_to_execute

# Goto action - Reference frame for the action
string goto_reference_frame_name

# Goto action - The position to which the goal stance is aligned
geometry_msgs/Point goto_goal_stance_point

# Goto action - The point that the robot should be facing in the goal stance
geometry_msgs/Point goto_goal_focal_point