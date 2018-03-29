# Message used to configure the exploration for a RRT-based planner.
# Main usage is the IHMC WholeBodyTrajectoryToolbox.

byte CONFIGURATION_SPACE_NAME_X = 0
byte CONFIGURATION_SPACE_NAME_Y = 1
byte CONFIGURATION_SPACE_NAME_Z = 2
byte CONFIGURATION_SPACE_NAME_ROLL = 3
byte CONFIGURATION_SPACE_NAME_PITCH = 4
byte CONFIGURATION_SPACE_NAME_YAW = 5

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

int64 rigid_body_name_based_hash_code
byte[] configuration_space_names_to_explore

float64[] exploration_range_upper_limits
float64[] exploration_range_lower_limits
