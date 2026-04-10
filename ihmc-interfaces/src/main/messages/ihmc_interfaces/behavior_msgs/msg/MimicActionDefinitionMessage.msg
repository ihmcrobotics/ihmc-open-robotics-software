# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

byte POLICY_TRANSITION = 0
byte EXIT_POLICY = 1
byte EXECUTE_POLICY = 2

# The type of action as defined above
byte mimic_action_type

# The name of the mimic file
string mimic_file_name

# Wait time in seconds after the policy has been exited
float64 wait_time_exit_policy
