byte FRONT_LEFT = 0
byte FRONT_RIGHT = 1
byte HIND_RIGHT = 2
byte HIND_LEFT = 3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte robot_quadrant 255
geometry_msgs/Point goal_position
float64 ground_clearance
