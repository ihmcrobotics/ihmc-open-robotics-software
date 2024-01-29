# Shape of the object
byte SHAPE_BOX = 0
byte SHAPE_PRISM = 1
byte SHAPE_ELLIPSOID = 2
byte SHAPE_CYLINDER = 3
byte SHAPE_CONE = 4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Scene node ID
uint32 node_id

# Shape of the object
byte shape 

# Dimensions of the object
geometry_msgs/Vector3 lengths
geometry_msgs/Vector3 radii

# User defined pose of virtual object
geometry_msgs/Pose provided_pose

# ICP parameters
int32 number_of_shape_samples
int32 number_of_correspondences
int32 number_of_iterations
float32 segmentation_radius

# Start/stop flag (true = run, false = stop)
bool run_icp

# Whether to use the user defined pose, or ICP objects centroid
bool use_provided_pose