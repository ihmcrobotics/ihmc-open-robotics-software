# Message used to configure the exploration for a RRT-based planner.
# Main usage is the IHMC WholeBodyTrajectoryToolbox.

byte CONFIGURATION_SPACE_NAME_X = 0
byte CONFIGURATION_SPACE_NAME_Y = 1
byte CONFIGURATION_SPACE_NAME_Z = 2
byte CONFIGURATION_SPACE_NAME_ROLL = 3
byte CONFIGURATION_SPACE_NAME_PITCH = 4
byte CONFIGURATION_SPACE_NAME_YAW = 5

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int64 rigid_body_name_based_hash_code
byte[] configuration_space_names_to_explore

float64[] exploration_range_upper_limits
float64[] exploration_range_lower_limits
