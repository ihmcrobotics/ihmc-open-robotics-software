# This message is part of the IHMC Robot Environment Awareness (REA) module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

float64 search_radius 0.05
float64 max_distance_from_plane 0.05
float64 max_angle_from_plane 0.1745
float64 min_normal_quality 0.005
int32 min_region_size 50
float64 max_standard_deviation 0.015
float64 min_volumic_density 100000  # In units of voxels per m^3. Default value is equivalent to 0.1 / cm^3