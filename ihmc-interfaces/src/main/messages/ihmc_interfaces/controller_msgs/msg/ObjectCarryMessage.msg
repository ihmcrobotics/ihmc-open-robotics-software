byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Specifies the side of the robot carrying the object
byte robot_side 255

# If true, will increase mass in the hand
bool is_picking_up

# Mass of the object being picked up
float64 object_mass

# World-frame offset from hand control frame to bag com
geometry_msgs/Vector3 hand_to_bag_com_offset