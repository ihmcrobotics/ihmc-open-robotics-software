# This message is part of the IHMC Robot Environment Awareness (REA) module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 0.05
float64 search_radius

# Field default value 0.05
float64 max_distance_from_plane

# Field default value 0.1745
float64 max_angle_from_plane

# Field default value 0.005
float64 min_normal_quality

# Field default value 50
int32 min_region_size

# Field default value 0.015
float64 max_standard_deviation

# In units of voxels per m^3. Default value is equivalent to 0.1 / cm^3
# Field default value 100000.0
float64 min_volumic_density


