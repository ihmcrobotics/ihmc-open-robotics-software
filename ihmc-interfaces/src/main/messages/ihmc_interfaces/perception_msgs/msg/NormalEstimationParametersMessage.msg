# This message is part of the IHMC Robot Environment Awareness (REA) module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

float64 search_radius 0.08
float64 max_distance_from_plane 0.02
float64 min_consensus_ratio 0.5
float64 max_average_deviation_ratio 0.75
int32 number_of_iterations 1
bool enable_least_squares_estimation true
bool weight_by_number_of_hits true