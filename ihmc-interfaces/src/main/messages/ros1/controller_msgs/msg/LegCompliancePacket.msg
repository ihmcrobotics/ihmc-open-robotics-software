int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# maximum allowed force (ratio) from velocity control in the range of [0.0, 1.0]. 1.0 is the
# maximum stiffness (default) value tuned for fast walking, 0.0 refers to zero velocity control
# making the joint very compliant (only force control) but often bad tracking. On Atlas, 0.1-0.3
# gives decent tracking for slow motion and yet still compliant. The numbers in the array
# correspond to joints HPZ, HPX, HPY, KNY, AKY, AKX, respectively.
float32[] max_velocity_deltas

# Field default value 255
int8 robot_side


