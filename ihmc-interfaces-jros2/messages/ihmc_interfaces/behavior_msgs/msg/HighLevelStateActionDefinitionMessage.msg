# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# The requested controller state, as a us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName ordinal
byte high_level_controller_name

# Seconds to wait for the controller to report the requested state before the action fails
float64 timeout

# With RL_CONTROL, the RL model to run, by its name in controller_msgs/RLPolicyState available_models. Empty keeps the current model.
string rl_model_name

# When true, the arms, spine and neck hold their current pose in the new state instead of moving to its home pose
bool keep_upper_body_pose
