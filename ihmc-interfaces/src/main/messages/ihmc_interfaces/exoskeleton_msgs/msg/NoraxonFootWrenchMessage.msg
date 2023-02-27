#This message is part of the Noraxon MR3 to Eva Exoskeleton Interface
#This message acts as a go-between between the Noraxon HTTP intepreter program and the Eva Controller 


byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Which side the Wrench is on
byte robot_side

# This is the wrench at the foot, with respect to the sole frame
float64 linear_force_x
float64 linear_force_y
float64 linear_force_z
float64 angular_moment_x
float64 angular_moment_y
float64 angular_moment_z

