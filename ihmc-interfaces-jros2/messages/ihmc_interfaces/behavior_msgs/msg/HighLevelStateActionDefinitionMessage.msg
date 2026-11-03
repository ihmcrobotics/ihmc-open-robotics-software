# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# The requested controller state, as a us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName ordinal
byte high_level_controller_name

# Seconds to wait for the controller to report the requested state before the action fails
float64 timeout

# With RL_CONTROL, the RL model to run, by its name in controller_msgs/RLPolicyState available_models. Empty keeps the current model.
string rl_model_name
