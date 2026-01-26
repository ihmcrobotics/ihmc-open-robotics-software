# Parent definition fields
behavior_msgs/LeafNodeDefinitionMessage definition

byte COUNTER_TYPE = 0
byte LLM_TYPE = 1
byte PROXIMITY_TYPE = 2
byte SHAPE_CONTAINS = 3
byte ALWAYS_FAIL = 4
byte ALWAYS_SUCCEED = 5

# The type of condition as defined above
byte type

# COUNTER TYPE

# The number of times to fail before passing
uint32 count_to

# LLM TYPE

# Whether to reset the context on each run
bool reset_context_each_run

# Whether to generate behavior state information and add it to the prompt
bool inject_behavior_state

# Whether to generate environment state information and add it to the prompt
bool inject_environment_state

# If a response match indicates success, else a match indicates failure
bool match_is_success

# The prompt given once at the very beginning
string<=10000 system

# The prompt defining how to make the boolean decision
string<=10000 prompt

# A regular expression, where if it matches, the execution failed
string<=10000 response_matcher

# PROXIMITY TYPE

byte XYZ = 0
byte XY = 1
byte Z = 2

# The type of distance condition as defined above
byte distance_type

# Name of frame A
string frame_name_a

# Name of frame B
string frame_name_b

# The minimum distance between the two frames
float64 min_distance

# The maximum distance between the two frames
float64 max_distance

# Timeout for waiting for the condition to be satisfied
float64 timeout

# SHAPE CONTAINS TYPE

byte CONTAINS_FRAME = 0
byte CONTAINS_POINTS = 1

# The type of shape contains condition as defined above
byte contains_type

# Name of the frame the the shape's pose is expressed in
string shape_parent_frame_name

# Transform that expresses the pose of the shape in the parent frame
controller_msgs/RigidBodyTransformMessage shape_transform_to_parent

# Radius of the sphere used for checking containment
float32 sphere_radius

# Name of frame to check for containment
string frame_name

# Minimum number of points in the shape
uint32 min_points
