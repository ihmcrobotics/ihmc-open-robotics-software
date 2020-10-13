# This message is part of the IHMC Robot Environment Awareness (REA) module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Threshold used when creating a new concave hull.
# Uses the Duckham and al. (2008) algorithm defined in the paper "Efficient generation of
# simple polygons for characterizing the shape of a set of points in the plane".
# Field default value 0.15
float64 concave_hull_threshold

# The minimum number of nodes required for a region to be polygonized.
# Field default value 10
int32 min_number_of_nodes

# Filter parameter on the concave hull of a region. Used to removed vertices describing shallow angle.
# Field default value 0.01745
float64 shallow_angle_threshold

# Filter parameter on the concave hull of a region. Used to removed vertices that create peaks.
# Field default value 2.967
float64 peak_angle_threshold

# Filter parameter on the concave hull of a region. Used to removed short edges.
# Field default value 0.05
float64 length_threshold

# Threshold used for decomposing the concave hull into convex polygons. Describes the maximum depth
# of a concavity before the concave hull gets split in 2.
# Field default value 0.1
float64 depth_threshold

# Filter for splitting concave hulls at any narrow passage which width is less than (2 * length_threshold).
# Field default value True
bool cut_narrow_passage


