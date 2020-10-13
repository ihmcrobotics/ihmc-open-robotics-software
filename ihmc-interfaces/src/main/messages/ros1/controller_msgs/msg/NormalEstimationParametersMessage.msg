# This message is part of the IHMC Robot Environment Awareness (REA) module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 0.08
float64 search_radius

# Field default value 0.02
float64 max_distance_from_plane

# Field default value 0.5
float64 min_consensus_ratio

# Field default value 0.75
float64 max_average_deviation_ratio

# Field default value 1
int32 number_of_iterations

# Field default value True
bool enable_least_squares_estimation

# Field default value True
bool weight_by_number_of_hits


