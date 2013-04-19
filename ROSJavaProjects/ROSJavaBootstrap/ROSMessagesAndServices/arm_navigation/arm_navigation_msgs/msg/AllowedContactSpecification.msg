# The names of the regions
string name

# The shape of the region in the environment
arm_navigation_msgs/Shape shape

# The pose of the space defining the region
geometry_msgs/PoseStamped pose_stamped

# The set of links that will be allowed to have penetration contact within this region
string[] link_names

# The maximum penetration depth allowed for every link
float64 penetration_depth
