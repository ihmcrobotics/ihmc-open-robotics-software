int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the side of the robot that will execute the command.
# Field default value 255
int8 robot_side

# The desired joint acceleration information.
controller_msgs/DesiredAccelerationsMessage desired_accelerations


