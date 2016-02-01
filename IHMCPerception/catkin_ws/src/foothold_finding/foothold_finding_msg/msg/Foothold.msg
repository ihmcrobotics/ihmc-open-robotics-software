# Header (time and frame)
Header header

# The step number of the foothold.
# Multiple alternative footholds per step number are possible.
uint8 stepNumber

# Foot type identifier (left, right etc.)
std_msgs/String type

# Footstep position and orientation
geometry_msgs/Pose pose

# Status flag (0: unknown, 1: do not change position, 2: verified, 3: bad)
int8 flag
