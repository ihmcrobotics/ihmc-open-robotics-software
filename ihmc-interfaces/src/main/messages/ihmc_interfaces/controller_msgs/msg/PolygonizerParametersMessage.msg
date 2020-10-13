# This message is part of the IHMC Robot Environment Awareness (REA) module.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Threshold used when creating a new concave hull.
# Uses the Duckham and al. (2008) algorithm defined in the paper "Efficient generation of
# simple polygons for characterizing the shape of a set of points in the plane".
float64 concave_hull_threshold 0.15

# The minimum number of nodes required for a region to be polygonized.
int32 min_number_of_nodes 10

# Filter parameter on the concave hull of a region. Used to removed vertices describing shallow angle.
float64 shallow_angle_threshold 0.01745

# Filter parameter on the concave hull of a region. Used to removed vertices that create peaks.
float64 peak_angle_threshold 2.967

# Filter parameter on the concave hull of a region. Used to removed short edges.
float64 length_threshold 0.05

# Threshold used for decomposing the concave hull into convex polygons. Describes the maximum depth
# of a concavity before the concave hull gets split in 2.
float64 depth_threshold 0.10

# Filter for splitting concave hulls at any narrow passage which width is less than (2 * length_threshold).
bool cut_narrow_passage true