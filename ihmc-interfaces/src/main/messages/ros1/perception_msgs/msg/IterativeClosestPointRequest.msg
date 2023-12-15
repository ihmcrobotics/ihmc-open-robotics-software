
# Shape of the object
int8 SHAPE_BOX=0

int8 SHAPE_PRISM=1

int8 SHAPE_ELLIPSOID=2

int8 SHAPE_CYLINDER=3

int8 SHAPE_CONE=4

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Scene node ID
uint32 node_id

# Shape of the object
int8 shape

# Dimensions of the object
float32 x_length

float32 y_length

float32 z_length

float32 x_radius

float32 y_radius

float32 z_radius

# User defined pose of virtual object
geometry_msgs/Pose provided_pose

# Start/stop flag (true = run, false = stop)
bool run_icp

# Whether to use the user defined pose, or ICP objects centroid
bool use_provided_pose


