int8 CONFIGURATION_SPACE_NAME_X=0

int8 CONFIGURATION_SPACE_NAME_Y=1

int8 CONFIGURATION_SPACE_NAME_Z=2

int8 CONFIGURATION_SPACE_NAME_ROLL=3

int8 CONFIGURATION_SPACE_NAME_PITCH=4

int8 CONFIGURATION_SPACE_NAME_YAW=5

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

int64 rigid_body_name_based_hash_code

int8[] configuration_space_names_to_explore

float64[] exploration_range_upper_limits

float64[] exploration_range_lower_limits


